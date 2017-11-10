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
		tiny_scene.views[0].reset(new View("", 0, 1, 0, imageL.Width(), imageL.Height()));
		tiny_scene.views[1].reset(new View("", 1, 1, 1, imageR.Width(), imageR.Height()));
		tiny_scene.intrinsics[0].reset(new Pinhole_Intrinsic(imageL.Width(), imageL.Height(), K(0, 0), K(0, 2), K(1, 2)));

		const Pinhole_Intrinsic * cam_I = dynamic_cast<const Pinhole_Intrinsic*> (tiny_scene.intrinsics[0].get());
		const Pinhole_Intrinsic * cam_J = dynamic_cast<const Pinhole_Intrinsic*> (tiny_scene.intrinsics[0].get());

		if (robustRelativePose(K, K, xL, xR, relativePose_info, size_imaL, size_imaL, 256))
		{
			std::vector<float> vec_angles;
			vec_angles.reserve(relativePose_info.vec_inliers.size());
			const Pose3 pose_I = Pose3(Mat3::Identity(), Vec3::Zero());
			const Pose3 pose_J = relativePose_info.relativePose;
			const Mat34 PI = tiny_scene.intrinsics[tiny_scene.views[0]->id_intrinsic]->get_projective_equivalent(pose_I);
			const Mat34 PJ = tiny_scene.intrinsics[tiny_scene.views[0]->id_intrinsic]->get_projective_equivalent(pose_J);

			for (const uint32_t & inlier_idx : relativePose_info.vec_inliers)
			{
				Vec3 X;
				TriangulateDLT(
					PI, xI.col(inlier_idx).homogeneous(),
					PJ, xJ.col(inlier_idx).homogeneous(), &X);

				openMVG::tracks::STLMAPTracks::const_iterator iterT = map_tracksCommon.begin();
				std::advance(iterT, inlier_idx);
				tracks::submapTrack::const_iterator iter = iterT->second.begin();
				const Vec2 featI = features_provider_->feats_per_view[I][iter->second].coords().cast<double>();
				const Vec2 featJ = features_provider_->feats_per_view[J][(++iter)->second].coords().cast<double>();
				vec_angles.push_back(AngleBetweenRay(pose_I, cam_I, pose_J, cam_J, featI, featJ));
			}
		}

		const Pose3 pose_I = sfm_data_.poses[view_I->id_pose] = tiny_scene.poses[view_I->id_pose];
		const Pose3 pose_J = sfm_data_.poses[view_J->id_pose] = tiny_scene.poses[view_J->id_pose];
		map_ACThreshold_.insert({ I, relativePose_info.found_residual_precision });
		map_ACThreshold_.insert({ J, relativePose_info.found_residual_precision });
		set_remaining_view_id_.erase(view_I->id_view);
		set_remaining_view_id_.erase(view_J->id_view);

		// List inliers and save them
		for (Landmarks::const_iterator iter = tiny_scene.GetLandmarks().begin();
			iter != tiny_scene.GetLandmarks().end(); ++iter)
		{
			const IndexT trackId = iter->first;
			const Landmark & landmark = iter->second;
			const Observations & obs = landmark.obs;
			Observations::const_iterator iterObs_xI = obs.find(view_I->id_view);
			Observations::const_iterator iterObs_xJ = obs.find(view_J->id_view);

			const Observation & ob_xI = iterObs_xI->second;
			const Observation & ob_xJ = iterObs_xJ->second;

			const double angle = AngleBetweenRay(
				pose_I, cam_I, pose_J, cam_J, ob_xI.x, ob_xJ.x);
			const Vec2 residual_I = cam_I->residual(pose_I, landmark.X, ob_xI.x);
			const Vec2 residual_J = cam_J->residual(pose_J, landmark.X, ob_xJ.x);
			if (angle > 2.0 &&
				pose_I.depth(landmark.X) > 0 &&
				pose_J.depth(landmark.X) > 0 &&
				residual_I.norm() < relativePose_info.found_residual_precision &&
				residual_J.norm() < relativePose_info.found_residual_precision)
			{
				sfm_data_.structure[trackId] = landmarks[trackId];
			}
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
		std::transform(sfm_data_.GetLandmarks().begin(), sfm_data_.GetLandmarks().end(),
			std::inserter(reconstructed_trackId, reconstructed_trackId.begin()),
			stl::RetrieveKey());

		// Get the ids of the already reconstructed tracks
		std::set<uint32_t> set_trackIdForResection;
		std::set_intersection(set_tracksIds.cbegin(), set_tracksIds.cend(),
			reconstructed_trackId.cbegin(), reconstructed_trackId.cend(),
			std::inserter(set_trackIdForResection, set_trackIdForResection.begin()));

		if (set_trackIdForResection.empty())
		{
			// No match. The image has no connection with already reconstructed points.
			std::cout << std::endl
				<< "-------------------------------" << "\n"
				<< "-- Resection of camera index: " << viewIndex << "\n"
				<< "-- Resection status: " << "FAILED" << "\n"
				<< "-------------------------------" << std::endl;
			return false;
		}

		// Get back featId associated to a tracksID already reconstructed.
		// These 2D/3D associations will be used for the resection.
		std::vector<uint32_t> vec_featIdForResection;
		TracksUtilsMap::GetFeatIndexPerViewAndTrackId(map_tracksCommon,
			set_trackIdForResection,
			viewIndex,
			&vec_featIdForResection);

		// Localize the image inside the SfM reconstruction
		Image_Localizer_Match_Data resection_data;
		resection_data.pt2D.resize(2, set_trackIdForResection.size());
		resection_data.pt3D.resize(3, set_trackIdForResection.size());

		// B. Look if intrinsic data is known or not
		const View * view_I = sfm_data_.GetViews().at(viewIndex).get();
		std::shared_ptr<cameras::IntrinsicBase> optional_intrinsic(nullptr);
		if (sfm_data_.GetIntrinsics().count(view_I->id_intrinsic))
		{
			optional_intrinsic = sfm_data_.GetIntrinsics().at(view_I->id_intrinsic);
		}

		// Setup the track 2d observation for this new view
		Mat2X pt2D_original(2, set_trackIdForResection.size());
		std::set<uint32_t>::const_iterator iterTrackId = set_trackIdForResection.begin();
		std::vector<uint32_t>::const_iterator iterfeatId = vec_featIdForResection.begin();
		for (size_t cpt = 0; cpt < vec_featIdForResection.size(); ++cpt, ++iterTrackId, ++iterfeatId)
		{
			resection_data.pt3D.col(cpt) = sfm_data_.GetLandmarks().at(*iterTrackId).X;
			resection_data.pt2D.col(cpt) = pt2D_original.col(cpt) =
				features_provider_->feats_per_view.at(viewIndex)[*iterfeatId].coords().cast<double>();
			// Handle image distortion if intrinsic is known (to ease the resection)
			if (optional_intrinsic && optional_intrinsic->have_disto())
			{
				resection_data.pt2D.col(cpt) = optional_intrinsic->get_ud_pixel(resection_data.pt2D.col(cpt));
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
