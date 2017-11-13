// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2016 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/image_describer_akaze_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include "openMVG/geometry/pose3.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/pipelines/localization/SfM_Localizer.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_filters.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/multiview/triangulation.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;
using namespace openMVG::cameras;

using namespace openMVG::image;
using namespace openMVG::matching_image_collection;
using namespace std;

namespace coloc
{
	class matcher {
	public:
		matcher(std::string& featType, std::string& dirPath);
		
		Pair_Set handlePairs();
		PairWiseMatches computePairMatches(std::string& imagePath1, std::string& imagePath2);

	private:
		std::unique_ptr<openMVG::features::Regions> regions_type;
		std::string baseDir;

		std::string featFile, descFile;

		PairWiseMatches map_PutativesMatches;
		std::map<Pair, unsigned int> overlap;
	};

	matcher::matcher(std::string& featType, std::string& dirPath) {

		baseDir = dirPath;

		if (featType == "AKAZE")
			regions_type.reset(new openMVG::features::AKAZE_Binary_Regions);
		else if (featType == "SIFT")
			regions_type.reset(new openMVG::features::SIFT_Regions);
	}

	PairWiseMatches matcher::computePairMatches(std::string& imagePath1, std::string& imagePath2) {
		featFile = stlplus::create_filespec(baseDir.c_str(), imagePath1.c_str(), ".feat");
		descFile = stlplus::create_filespec(baseDir.c_str(), imagePath1.c_str(), ".desc");

		std::unique_ptr<features::Regions> regions_ptr1(regions_type->EmptyClone());
		std::unique_ptr<features::Regions> regions_ptr2(regions_type->EmptyClone());
		if (!regions_ptr1->Load(featFile, descFile))
			std::cerr << "Invalid regions files for the view: " << std::endl;

		featFile = stlplus::create_filespec(baseDir.c_str(), imagePath2.c_str(), ".feat");
		descFile = stlplus::create_filespec(baseDir.c_str(), imagePath2.c_str(), ".desc");

		if (!regions_ptr2->Load(featFile, descFile))
			std::cerr << "Invalid regions files for the view: " << std::endl;

		std::vector<IndMatch> vec_PutativeMatches;

		matching::DistanceRatioMatch(
			0.8, matching::BRUTE_FORCE_HAMMING,
			*regions_ptr1.get(),
			*regions_ptr2.get(),
			vec_PutativeMatches);

		if (!vec_PutativeMatches.empty()) {
			map_PutativesMatches.insert({ { 0,1 }, std::move(vec_PutativeMatches) });
			overlap.insert({ { 0, 1 }, std::move(vec_PutativeMatches.size()) });
		}
		else
			overlap.insert({ { 0, 1 }, 0 });
	
		return map_PutativesMatches;
	}
}

int main(int argc, char **argv)
{
	CmdLine cmd;
		
	using namespace openMVG::features;
	std::unique_ptr<Regions> regions_type;
	std::map<IndexT, std::unique_ptr<features::Regions> > regions;

	const std::string sInputDir = "C:\\Users\\svempral\\Desktop\\test\\";

	const string jpg_filenameL = sInputDir + "img__Quad0_0.png";
	const string jpg_filenameM = sInputDir + "img__Quad1_0.png";
	const string jpg_filenameR = sInputDir + "img__Quad2_0.png";

	Image<unsigned char> imageL, imageR, imageM;
	ReadImage(jpg_filenameL.c_str(), &imageL);
	ReadImage(jpg_filenameM.c_str(), &imageM);
	ReadImage(jpg_filenameR.c_str(), &imageR);

	using namespace openMVG::features;
	std::unique_ptr<Image_describer> image_describer(new AKAZE_Image_describer_MLDB);
	std::map<IndexT, std::unique_ptr<features::Regions>> regions_perImage;
	image_describer->Describe(imageL, regions[0]);
	image_describer->Describe(imageM, regions[1]);
	image_describer->Describe(imageR, regions[2]);

	const AKAZE_Binary_Regions* regionsL = dynamic_cast<AKAZE_Binary_Regions*>(regions.at(0).get());
	const AKAZE_Binary_Regions* regionsM = dynamic_cast<AKAZE_Binary_Regions*>(regions.at(1).get());
	const AKAZE_Binary_Regions* regionsR = dynamic_cast<AKAZE_Binary_Regions*>(regions.at(2).get());


	const PointFeatures
		featsL = regions.at(0)->GetRegionsPositions(),
		featsM = regions.at(1)->GetRegionsPositions(),
		featsR = regions.at(2)->GetRegionsPositions();

	Pair_Set pairs = exhaustivePairs(3);

	PairWiseMatches map_PutativesMatches;

	for (const auto & pair_idx : pairs)
	{

		std::vector<IndMatch> vec_PutativeMatches;
		matching::DistanceRatioMatch(0.8, matching::BRUTE_FORCE_HAMMING, *regions.at(pair_idx.first).get(), *regions.at(pair_idx.second).get(), vec_PutativeMatches);
		if (!vec_PutativeMatches.empty()) {
			map_PutativesMatches.insert({ { pair_idx.first, pair_idx.second }, std::move(vec_PutativeMatches) });
		}
	}

	for (const std::pair< Pair, IndMatches > & match_pair : map_PutativesMatches) {
		const Pair current_pair = match_pair.first;

		const uint32_t I = std::min(current_pair.first, current_pair.second);
		const uint32_t J = std::max(current_pair.first, current_pair.second);

		const PointFeatures featI = regions.at(I)->GetRegionsPositions();
		const PointFeatures featJ = regions.at(J)->GetRegionsPositions();

		std::vector <IndMatch> putativeMatches = match_pair.second;
		Mat xL(2, putativeMatches.size());
		Mat xR(2, putativeMatches.size());
		for (size_t k = 0; k < putativeMatches.size(); ++k) {
			xL.col(k) = featI[k].coords().cast<double>();
			xR.col(k) = featJ[k].coords().cast<double>();
		}


		Mat3 K;
		K << 320, 0, 320,
			0, 320, 240,
			0, 0, 1;

		std::pair<size_t, size_t> size_imaL(640, 480);

		RelativePose_Info relativePose_info;
		SfM_Data tiny_scene;
		tiny_scene.views[0].reset(new View("", 0, 0, 0, imageL.Width(), imageL.Height()));
		tiny_scene.views[1].reset(new View("", 1, 0, 1, imageR.Width(), imageR.Height()));
		tiny_scene.intrinsics[0].reset(new Pinhole_Intrinsic(imageL.Width(), imageL.Height(), K(0, 0), K(0, 2), K(1, 2)));

		if (!robustRelativePose(K, K, xL, xR, relativePose_info, size_imaL, size_imaL, 256))
		{
			std::cerr << " /!\\ Robust relative pose estimation failure."
				<< std::endl;
			return EXIT_FAILURE;
		}

		std::cout << "\nFound an Essential matrix:\n"
			<< "\tprecision: " << relativePose_info.found_residual_precision << " pixels\n"
			<< "\t#inliers: " << relativePose_info.vec_inliers.size() << "\n"
			<< "\t#matches: " << putativeMatches.size()
			<< std::endl;


		const Pinhole_Intrinsic * cam_I = dynamic_cast<const Pinhole_Intrinsic*> (tiny_scene.intrinsics[0].get());
		const Pinhole_Intrinsic * cam_J = dynamic_cast<const Pinhole_Intrinsic*> (tiny_scene.intrinsics[0].get());

		const Pose3 pose0 = tiny_scene.poses[tiny_scene.views[0]->id_pose] = Pose3(Mat3::Identity(), Vec3::Zero());
		const Pose3 pose1 = tiny_scene.poses[tiny_scene.views[1]->id_pose] = relativePose_info.relativePose;

		// Init structure by inlier triangulation
		const Mat34 P1 = tiny_scene.intrinsics[tiny_scene.views[0]->id_intrinsic]->get_projective_equivalent(pose0);
		const Mat34 P2 = tiny_scene.intrinsics[tiny_scene.views[1]->id_intrinsic]->get_projective_equivalent(pose1);
		Landmarks & landmarks = tiny_scene.structure;
		for (size_t i = 0; i < relativePose_info.vec_inliers.size(); ++i) {
			const SIOPointFeature & LL = regionsL->Features()[putativeMatches[relativePose_info.vec_inliers[i]].i_];
			const SIOPointFeature & RR = regionsM->Features()[putativeMatches[relativePose_info.vec_inliers[i]].j_];
			// Point triangulation
			Vec3 X;
			TriangulateDLT(
				P1, LL.coords().cast<double>().homogeneous(),
				P2, RR.coords().cast<double>().homogeneous(), &X);
			// Reject point that is behind the camera
			if (pose0.depth(X) < 0 && pose1.depth(X) < 0)
				continue;
			// Add a new landmark (3D point with it's 2d observations)
			landmarks[i].obs[tiny_scene.views[0]->id_view] = Observation(LL.coords().cast<double>(), putativeMatches[relativePose_info.vec_inliers[i]].i_);
			landmarks[i].obs[tiny_scene.views[1]->id_view] = Observation(RR.coords().cast<double>(), putativeMatches[relativePose_info.vec_inliers[i]].j_);
			landmarks[i].X = X;
		}

		tracks::TracksBuilder tracksBuilder;
		const openMVG::matching::PairWiseMatches & map_Matches = map_PutativesMatches;
		tracksBuilder.Build(map_Matches);
		openMVG::tracks::STLMAPTracks map_tracks_;
		tracksBuilder.ExportToSTL(map_tracks_);

		std::unique_ptr<openMVG::tracks::SharedTrackVisibilityHelper> shared_track_visibility_helper_;
		shared_track_visibility_helper_.reset(new openMVG::tracks::SharedTrackVisibilityHelper(map_tracks_));



		openMVG::tracks::STLMAPTracks map_tracksCommon;
		shared_track_visibility_helper_->GetTracksInImages({ 2 }, map_tracksCommon);
		std::set<uint32_t> set_tracksIds;
		openMVG::tracks::TracksUtilsMap::GetTracksIdVector(map_tracksCommon, &set_tracksIds);

		std::set<uint32_t> reconstructed_trackId;
		std::transform(tiny_scene.GetLandmarks().begin(), tiny_scene.GetLandmarks().end(),
			std::inserter(reconstructed_trackId, reconstructed_trackId.begin()),
			stl::RetrieveKey());

		std::set<uint32_t> set_trackIdForResection;
		std::set_intersection(set_tracksIds.cbegin(), set_tracksIds.cend(),
			reconstructed_trackId.cbegin(), reconstructed_trackId.cend(),
			std::inserter(set_trackIdForResection, set_trackIdForResection.begin()));

		Hash_Map<IndexT, double> map_ACThreshold_;

		if (set_trackIdForResection.empty())
		{
			// No match. The image has no connection with already reconstructed points.
			std::cout << std::endl
				<< "-------------------------------" << "\n"
				<< "-- Resection of camera index: " << "\n"
				<< "-- Resection status: " << "FAILED" << "\n"
				<< "-------------------------------" << std::endl;
			return false;
		}

		std::vector<uint32_t> vec_featIdForResection;
		openMVG::tracks::TracksUtilsMap::GetFeatIndexPerViewAndTrackId(map_tracksCommon,
			set_trackIdForResection,
			2,
			&vec_featIdForResection);

		Image_Localizer_Match_Data resection_data;
		resection_data.pt2D.resize(2, set_trackIdForResection.size());
		resection_data.pt3D.resize(3, set_trackIdForResection.size());

		tiny_scene.views[2].reset(new View("", 2, 0, 1, imageR.Width(), imageR.Height()));

		const View * view_I = tiny_scene.GetViews().at(2).get();
		std::shared_ptr<cameras::IntrinsicBase> optional_intrinsic(nullptr);
		if (tiny_scene.GetIntrinsics().count(view_I->id_intrinsic))
		{
			optional_intrinsic = tiny_scene.GetIntrinsics().at(view_I->id_intrinsic);
		}

		// Setup the track 2d observation for this new view
		Mat2X pt2D_original(2, set_trackIdForResection.size());
		std::set<uint32_t>::const_iterator iterTrackId = set_trackIdForResection.begin();
		std::vector<uint32_t>::const_iterator iterfeatId = vec_featIdForResection.begin();
		for (size_t cpt = 0; cpt < vec_featIdForResection.size(); ++cpt, ++iterTrackId, ++iterfeatId)
		{
			resection_data.pt3D.col(cpt) = tiny_scene.GetLandmarks().at(*iterTrackId).X;
			resection_data.pt2D.col(cpt) = pt2D_original.col(cpt) =
				featsR[*iterfeatId].coords().cast<double>();
			// Handle image distortion if intrinsic is known (to ease the resection)
			if (optional_intrinsic && optional_intrinsic->have_disto())
			{
				resection_data.pt2D.col(cpt) = optional_intrinsic->get_ud_pixel(resection_data.pt2D.col(cpt));
			}
		}

		// C. Do the resectioning: compute the camera pose
		std::cout << std::endl
			<< "-------------------------------" << std::endl
			<< "-- Robust Resection of view: " << std::endl;

		geometry::Pose3 pose;
		const bool bResection = sfm::SfM_Localizer::Localize
		(
			optional_intrinsic ? resection::SolverType::P3P_KE_CVPR17 : resection::SolverType::DLT_6POINTS,
			{ view_I->ui_width, view_I->ui_height },
			optional_intrinsic.get(),
			resection_data,
			pose
		);
		resection_data.pt2D = std::move(pt2D_original); // restore original image domain points


			std::cout << std::endl
				<< "-------------------------------" << "<br>"
				<< "-- Robust Resection of camera index: <" << "> image: "
				<< view_I->s_Img_path << "<br>"
				<< "-- Threshold: " << resection_data.error_max << "<br>"
				<< "-- Resection status: " << (bResection ? "OK" : "FAILED") << "<br>"
				<< "-- Nb points used for Resection: " << vec_featIdForResection.size() << "<br>"
				<< "-- Nb points validated by robust estimation: " << resection_data.vec_inliers.size() << "<br>"
				<< "-- % points validated: "
				<< resection_data.vec_inliers.size() / static_cast<float>(vec_featIdForResection.size()) << "<br>"
				<< "-------------------------------" << "<br>";


			// E. Update the global scene with:
			// - the new found camera pose
			tiny_scene.poses[view_I->id_pose] = pose;
			// - track the view's AContrario robust estimation found threshold
			// map_ACThreshold_.insert({ viewIndex, resection_data.error_max });
			// - intrinsic parameters (if the view has no intrinsic group add a new one)
			

			map_ACThreshold_.insert({ 2, resection_data.error_max });
			{
				const IndexT I = 2;
				const View * view_I = tiny_scene.GetViews().at(I).get();
				const IntrinsicBase * cam_I = tiny_scene.GetIntrinsics().at(view_I->id_intrinsic).get();
				const Pose3 pose_I = tiny_scene.GetPoseOrDie(view_I);

				// Vector of all already reconstructed views
				const std::set<IndexT> valid_views = Get_Valid_Views(tiny_scene);

				// Go through each track and look if we must add new view observations or new 3D points
				for (const std::pair< uint32_t, tracks::submapTrack >& trackIt : map_tracksCommon)
				{
					const uint32_t trackId = trackIt.first;
					const tracks::submapTrack & track = trackIt.second;

					// List the potential view observations of the track
					const tracks::submapTrack & allViews_of_track = map_tracks_[trackId];

					// List to save the new view observations that must be added to the track
					std::set<IndexT> new_track_observations_valid_views;

					// If the track was already reconstructed
					if (tiny_scene.structure.count(trackId) != 0)
					{
						// Since the 3D point was triangulated before we add the new the Inth view observation
						new_track_observations_valid_views.insert(I);
					}
					else
					{
						// Go through the views that observe this track & look if a successful triangulation can be done
						for (const std::pair< IndexT, IndexT >& trackViewIt : allViews_of_track)
						{
							const IndexT & J = trackViewIt.first;
							// If view is valid try triangulation
							if (J != I && valid_views.count(J) != 0)
							{
								// If successfuly triangulated add the observation from J view
								if (tiny_scene.structure.count(trackId) != 0)
								{
									new_track_observations_valid_views.insert(J);
								}
								else
								{
									const View * view_J = tiny_scene.GetViews().at(J).get();
									const IntrinsicBase * cam_J = tiny_scene.GetIntrinsics().at(view_J->id_intrinsic).get();
									const Pose3 pose_J = tiny_scene.GetPoseOrDie(view_J);
									const Vec2 xJ = featsR[allViews_of_track.at(J)].coords().cast<double>();

									// Position of the point in view I
									const Vec2 xI = featsL[track.at(I)].coords().cast<double>();

									// Try to triangulate a 3D point from J view
									// A new 3D point must be added
									// Triangulate it
									const Vec2 xI_ud = cam_I->get_ud_pixel(xI);
									const Vec2 xJ_ud = cam_J->get_ud_pixel(xJ);
									const Mat34 P_I = cam_I->get_projective_equivalent(pose_I);
									const Mat34 P_J = cam_J->get_projective_equivalent(pose_J);
									Vec3 X = Vec3::Zero();
									TriangulateDLT(P_I, xI_ud.homogeneous(), P_J, xJ_ud.homogeneous(), &X);
									// Check triangulation result
									const double angle = AngleBetweenRay(pose_I, cam_I, pose_J, cam_J, xI, xJ);
									const Vec2 residual_I = cam_I->residual(pose_I, X, xI);
									const Vec2 residual_J = cam_J->residual(pose_J, X, xJ);
									if (
										//  - Check angle (small angle leads to imprecise triangulation)
										angle > 2.0 &&
										//  - Check positive depth
										pose_I.depth(X) > 0 &&
										pose_J.depth(X) > 0 &&
										//  - Check residual values (must be inferior to the found view's AContrario threshold)
										residual_I.norm() < std::max(4.0, map_ACThreshold_.at(I)) &&
										residual_J.norm() < std::max(4.0, map_ACThreshold_.at(J))
										)
									{
										// Add a new track
										Landmark & landmark = tiny_scene.structure[trackId];
										landmark.X = X;
										new_track_observations_valid_views.insert(I);
										new_track_observations_valid_views.insert(J);
									} // 3D point is valid
									else
									{
										// We mark the view to add the observations once the point is triangulated
										new_track_observations_valid_views.insert(J);
									} // 3D point is invalid
								}
							}
						}// Go through all the views
					}// If new point

					 // If successfuly triangulated, add the valid view observations
					if (tiny_scene.structure.count(trackId) != 0 &&
						!new_track_observations_valid_views.empty()
						)
					{
						Landmark & landmark = tiny_scene.structure[trackId];
						// Check if view feature point observations of the track are valid (residual, depth) or not
						for (const IndexT &J : new_track_observations_valid_views)
						{
							const View * view_J = tiny_scene.GetViews().at(J).get();
							const IntrinsicBase * cam_J = tiny_scene.GetIntrinsics().at(view_J->id_intrinsic).get();
							const Pose3 pose_J = tiny_scene.GetPoseOrDie(view_J);
							const Vec2 xJ = featsR[allViews_of_track.at(J)].coords().cast<double>();

							const Vec2 residual = cam_J->residual(pose_J, landmark.X, xJ);
							if (pose_J.depth(landmark.X) > 0 &&
								residual.norm() < std::max(4.0, map_ACThreshold_.at(J))
								)
							{
								landmark.obs[J] = Observation(xJ, allViews_of_track.at(J));
							}
						}
					}
				}

			}
	}


	/*
	Mat xL(2, vec_PutativeMatches.size());
	Mat xR(2, vec_PutativeMatches.size());
	for (size_t k = 0; k < vec_PutativeMatches.size(); ++k) {
		const PointFeature & imaL = featsL[vec_PutativeMatches[k].i_];
		const PointFeature & imaR = featsR[vec_PutativeMatches[k].j_];
		xL.col(k) = imaL.coords().cast<double>();
		xR.col(k) = imaR.coords().cast<double>();
	}

	Mat3 K;
	K << 320, 0, 320, 
		0, 320, 240, 
		0, 0, 1;

	//B. Compute the relative pose thanks to a essential matrix estimation
	std::pair<size_t, size_t> size_imaL(640, 480);
	std::pair<size_t, size_t> size_imaR(640, 480);
	RelativePose_Info relativePose_info;
	if (!robustRelativePose(K, K, xL, xR, relativePose_info, size_imaL, size_imaR, 256))
	{
		std::cerr << " /!\\ Robust relative pose estimation failure."
			<< std::endl;
		return EXIT_FAILURE;
	}
	
	std::cout << "\nFound an Essential matrix:\n"
		<< "\tprecision: " << relativePose_info.found_residual_precision << " pixels\n"
		<< "\t#inliers: " << relativePose_info.vec_inliers.size() << "\n"
		<< "\t#matches: " << vec_PutativeMatches.size()
		<< std::endl;
	*/
	return EXIT_SUCCESS;
}
