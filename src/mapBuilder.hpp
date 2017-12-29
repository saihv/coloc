#pragma once
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"
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

#include "localizationData.hpp"

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::sfm;

namespace coloc
{
	class Reconstructor {
	public:
		
		Reconstructor(int w, int h) : regionsRCT(nullptr), matchesRCT(nullptr), relativePosesRCT(nullptr), scene(nullptr)
		{
			imageSize.first = w;
			imageSize.second = w;
			K << 320, 0, 320,
				0, 320, 240,
				0, 0, 1;
		}

		void reconstructScene(LocalizationData& data);

	private:
		// Internal methods

		void initializeTracks(Pair& viewPair);
		void initializeScene(int w, int h);
		void triangulatePoints();
		void constructProjectionsTriangulation(Mat34& P1, Mat34& P2);
		bool resectionCamera(unsigned int);

		// Internal variables

		tracks::TracksBuilder tracksBuilder;
		openMVG::tracks::STLMAPTracks mapTracks;
		std::unique_ptr<openMVG::tracks::SharedTrackVisibilityHelper> trackVisibility;
		cameras::Pinhole_Intrinsic *cam_I, *cam_J;
		std::pair <size_t, size_t> imageSize;
		Mat3 K;

		// Local pointers to external localization data

		std::unique_ptr <std::map<IndexT, std::unique_ptr<features::Regions> > > regionsRCT;
		std::unique_ptr <PairWiseMatches> matchesRCT;
		std::unique_ptr <std::map<Pair, RelativePose_Info > > relativePosesRCT;
		std::unique_ptr <SfM_Data> scene;
	};
	
	void Reconstructor::reconstructScene(LocalizationData& data)
	{
		this->regionsRCT = std::make_unique <std::map <IndexT, std::unique_ptr<features::Regions> > >(data.regions);
		this->matchesRCT = std::make_unique <PairWiseMatches> (data.geometricMatches);
		this->relativePosesRCT = std::make_unique <std::map <Pair, RelativePose_Info> > (data.relativePoses);
		this->scene = std::make_unique <SfM_Data> (data.scene);

		int maxMatches = 0;
		Pair seedPair;

		for (auto matchedPair : *matchesRCT) {
			Pair currentPair = matchedPair.first;
			if (matchedPair.second.size() > maxMatches) {
				maxMatches = matchedPair.second.size();
				seedPair = currentPair;
			}
		}

		initializeTracks(seedPair);
		initializeScene(640, 480);
		triangulatePoints();	
	}
	
	void Reconstructor::initializeTracks(Pair& viewPair)
	{
		this->tracksBuilder.Build(*matchesRCT);
		this->tracksBuilder.Filter();
		this->tracksBuilder.ExportToSTL(mapTracks);
		this->trackVisibility.reset(new openMVG::tracks::SharedTrackVisibilityHelper(mapTracks));

		openMVG::tracks::STLMAPTracks map_tracksCommon;
		this->trackVisibility->GetTracksInImages({ viewPair.first, viewPair.second }, map_tracksCommon);
	}

	void Reconstructor::initializeScene(int w, int h)
	{
		this->scene->views[0].reset(new View("", 0, 0, 0, w, h));
		this->scene->views[1].reset(new View("", 1, 0, 1, w, h));

		this->scene->intrinsics[0].reset(new cameras::Pinhole_Intrinsic(w, h, K(0, 0), K(0, 2), K(1, 2)));

		this->cam_I = dynamic_cast<cameras::Pinhole_Intrinsic*> (scene->intrinsics[0].get());
		this->cam_J = dynamic_cast<cameras::Pinhole_Intrinsic*> (scene->intrinsics[0].get());
	}
	
	void Reconstructor::triangulatePoints()
	{
		const size_t n = mapTracks.size();
		Mat xI(2, n), xJ(2, n);
		uint32_t cptIndex = 0;
		Vec2 feat;
		for (openMVG::tracks::STLMAPTracks::const_iterator iterT = mapTracks.begin(); iterT != mapTracks.end(); ++iterT, ++cptIndex) {
			tracks::submapTrack::const_iterator iter = iterT->second.begin();
			const uint32_t i = iter->second;
			const uint32_t j = (++iter)->second;

			feat = regionsRCT->at(0)->GetRegionsPositions()[i].coords().cast<double>();
			xI.col(cptIndex) = cam_I->get_ud_pixel(feat);
			feat = regionsRCT->at(1)->GetRegionsPositions()[j].coords().cast<double>();
			xJ.col(cptIndex) = cam_J->get_ud_pixel(feat);
		}

		Landmarks & landmarks = scene->structure;

		for (openMVG::tracks::STLMAPTracks::const_iterator
			iterT = mapTracks.begin();
			iterT != mapTracks.end();
			++iterT)
		{
			// Get corresponding points
			tracks::submapTrack::const_iterator iter = iterT->second.begin();
			const uint32_t i = iter->second;
			const uint32_t j = (++iter)->second;

			const Vec2 x1_ = regionsRCT->at(0)->GetRegionsPositions()[i].coords().cast<double>();
			const Vec2 x2_ = regionsRCT->at(1)->GetRegionsPositions()[j].coords().cast<double>();

			Vec3 X;
			Mat34 P1, P2;
			constructProjectionsTriangulation(P1, P2);
			TriangulateDLT(P1, x1_.homogeneous(), P2, x2_.homogeneous(), &X);
			Observations obs;
			obs[scene->views[0]->id_view] = Observation(x1_, i);
			obs[scene->views[1]->id_view] = Observation(x2_, j);
			landmarks[iterT->first].obs = std::move(obs);
			landmarks[iterT->first].X = X;
		}
	}

	void Reconstructor::constructProjectionsTriangulation(Mat34& P1, Mat34& P2)
	{
		const Pose3 Pose_I = scene->poses[scene->views[0]->id_pose] = Pose3(Mat3::Identity(), Vec3::Zero());
		const Pose3 Pose_J = scene->poses[scene->views[1]->id_pose] = relativePosesRCT->at({ 0,1 }).relativePose;

		P1 = cam_I->get_projective_equivalent(Pose_I);
		P2 = cam_J->get_projective_equivalent(Pose_J);
	}
	
	bool Reconstructor::resectionCamera(unsigned int id)
	{
		openMVG::tracks::STLMAPTracks map_tracksCommonNew;
		trackVisibility->GetTracksInImages({ id }, map_tracksCommonNew);
		std::set<uint32_t> set_tracksIds;
		openMVG::tracks::TracksUtilsMap::GetTracksIdVector(map_tracksCommonNew, &set_tracksIds);

		std::set<uint32_t> reconstructed_trackId;
		std::transform(scene->GetLandmarks().begin(), scene->GetLandmarks().end(), std::inserter(reconstructed_trackId, reconstructed_trackId.begin()), stl::RetrieveKey());

		std::set<uint32_t> set_trackIdForResection;
		std::set_intersection(set_tracksIds.cbegin(), set_tracksIds.cend(), reconstructed_trackId.cbegin(), reconstructed_trackId.cend(), std::inserter(set_trackIdForResection, set_trackIdForResection.begin()));


		const PointFeatures resectionFeats = regionsRCT->at(id)->GetRegionsPositions();

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
		openMVG::tracks::TracksUtilsMap::GetFeatIndexPerViewAndTrackId(map_tracksCommonNew,
			set_trackIdForResection,
			2,
			&vec_featIdForResection);

		Image_Localizer_Match_Data resection_data;
		resection_data.pt2D.resize(2, set_trackIdForResection.size());
		resection_data.pt3D.resize(3, set_trackIdForResection.size());

		scene->views[id].reset(new View("", 2, 0, 2, imageSize.first, imageSize.second));

		const View * view_I = scene->GetViews().at(id).get();
		std::shared_ptr<cameras::IntrinsicBase> optional_intrinsic(nullptr);
		if (scene->GetIntrinsics().count(view_I->id_intrinsic))
			optional_intrinsic = scene->GetIntrinsics().at(view_I->id_intrinsic);

		// Setup the track 2d observation for this new view
		Mat2X pt2D_original(2, set_trackIdForResection.size());
		std::set<uint32_t>::const_iterator iterTrackId = set_trackIdForResection.begin();
		std::vector<uint32_t>::const_iterator iterfeatId = vec_featIdForResection.begin();
		for (size_t cpt = 0; cpt < vec_featIdForResection.size(); ++cpt, ++iterTrackId, ++iterfeatId)
		{
			resection_data.pt3D.col(cpt) = scene->GetLandmarks().at(*iterTrackId).X;
			resection_data.pt2D.col(cpt) = pt2D_original.col(cpt) = resectionFeats[*iterfeatId].coords().cast<double>();
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
			resection::SolverType::P3P_KE_CVPR17,
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


		scene->poses[view_I->id_pose] = pose;


		// Since the view have not yet an intrinsic group before, create a new one
		IndexT new_intrinsic_id = 0;
		if (!scene->GetIntrinsics().empty())
		{
			// Since some intrinsic Id already exists,
			//  we have to create a new unique identifier following the existing one
			std::set<IndexT> existing_intrinsicId;
			std::transform(scene->GetIntrinsics().begin(), scene->GetIntrinsics().end(),
				std::inserter(existing_intrinsicId, existing_intrinsicId.begin()),
				stl::RetrieveKey());
			new_intrinsic_id = (*existing_intrinsicId.rbegin()) + 1;
		}
		scene->views.at(id)->id_intrinsic = new_intrinsic_id;
		scene->intrinsics[new_intrinsic_id] = optional_intrinsic;



		// F. List tracks that share content with this view and add observations and new 3D track if required.
		//    - If the track already exists (look if the new view tracks observation are valid)
		//    - If the track does not exists, try robust triangulation & add the new valid view track observation
		{
			// Get information of new view
			const IndexT I = 2;
			const View * view_I = scene->GetViews().at(I).get();
			const cameras::IntrinsicBase * cam_I = scene->GetIntrinsics().at(view_I->id_intrinsic).get();
			const Pose3 pose_I = scene->GetPoseOrDie(view_I);

			// Vector of all already reconstructed views
			const std::set<IndexT> valid_views = Get_Valid_Views(*scene);

			// Go through each track and look if we must add new view observations or new 3D points
			for (const std::pair< uint32_t, tracks::submapTrack >& trackIt : map_tracksCommonNew)
			{
				const uint32_t trackId = trackIt.first;
				const tracks::submapTrack & track = trackIt.second;

				// List the potential view observations of the track
				const tracks::submapTrack & allViews_of_track = mapTracks[trackId];

				// List to save the new view observations that must be added to the track
				std::set<IndexT> new_track_observations_valid_views;

				// If the track was already reconstructed
				if (scene->structure.count(trackId) != 0)
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
							if (scene->structure.count(trackId) != 0)
							{
								new_track_observations_valid_views.insert(J);
							}
							else
							{
								const View * view_J = scene->GetViews().at(J).get();
								const cameras::IntrinsicBase * cam_J = scene->GetIntrinsics().at(view_J->id_intrinsic).get();
								const Pose3 pose_J = scene->GetPoseOrDie(view_J);
								const Vec2 xJ = resectionFeats[allViews_of_track.at(J)].coords().cast<double>();

								// Position of the point in view I
								const Vec2 xI = regionsRCT->at(I)->GetRegionsPositions()[track.at(I)].coords().cast<double>();

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
								const Vec2 residual_I = cam_I->residual(pose_I(X), xI);
								const Vec2 residual_J = cam_J->residual(pose_J(X), xJ);
								if (
									//  - Check angle (small angle leads to imprecise triangulation)
									angle > 2.0 &&
									//  - Check positive depth
									pose_I.depth(X) > 0 &&
									pose_J.depth(X) > 0
									)
								{
									// Add a new track
									Landmark & landmark = scene->structure[trackId];
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
				if (scene->structure.count(trackId) != 0 && !new_track_observations_valid_views.empty())
				{
					Landmark & landmark = scene->structure[trackId];
					// Check if view feature point observations of the track are valid (residual, depth) or not
					for (const IndexT &J : new_track_observations_valid_views)
					{
						const View * view_J = scene->GetViews().at(J).get();
						const cameras::IntrinsicBase * cam_J = scene->GetIntrinsics().at(view_J->id_intrinsic).get();
						const Pose3 pose_J = scene->GetPoseOrDie(view_J);
						const Vec2 xJ = resectionFeats[allViews_of_track.at(J)].coords().cast<double>();

						const Vec2 residual = cam_J->residual(pose_J(landmark.X), xJ);
						if (pose_J.depth(landmark.X) > 0)
						{
							landmark.obs[J] = Observation(xJ, allViews_of_track.at(J));
						}
					}
				}
			}// All the tracks in the view
		}
	}
}