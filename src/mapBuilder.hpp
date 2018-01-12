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
using namespace openMVG::geometry;

namespace coloc
{
	class Reconstructor {
	public:
		Reconstructor(LocalizationParams& params)
		{
			this->imageSize = &params.imageSize;
			this->K = &params.K;
			this->currentFolder = &params.imageFolder;
		}

		void reconstructScene(LocalizationData& data, Pose3, float);

	private:
		// Internal methods

		void initializeTracks(Pair& viewPair);
		void initializeScene(int w, int h);
		void triangulatePoints(Pose3&, float&);
		void constructProjectionsTriangulation(Pose3&, Pose3&, Mat34& P1, Mat34& P2);
		bool resectionCamera(unsigned int);
		bool saveSceneData(SfM_Data* scene);
		Pose3 relativePoseToAbsolute(Pose3&, Pose3&);

		// Internal variables

		tracks::TracksBuilder tracksBuilder;
		openMVG::tracks::STLMAPTracks mapTracks, mapTracksCommon;
		std::unique_ptr<openMVG::tracks::SharedTrackVisibilityHelper> trackVisibility;
		cameras::Pinhole_Intrinsic *camCurrent, *cam_J;
		std::pair <size_t, size_t> *imageSize;
		Mat3 *K;
		std::string *currentFolder;

		// Local pointers to external localization data

		std::map<IndexT, std::unique_ptr<features::Regions> > * regionsRCT;
		PairWiseMatches* matchesRCT;
		std::map<Pair, RelativePose_Info > * relativePosesRCT;
		SfM_Data* scene;
	};
	
	void Reconstructor::reconstructScene(LocalizationData& data, Pose3 origin = Pose3(Mat3::Identity(), Vec3::Zero()), float scale = 1.0)
	{
		this->regionsRCT = &data.regions;
		this->matchesRCT = &data.geometricMatches;
		this->relativePosesRCT = &data.relativePoses;
		this->scene = &data.scene;

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
		triangulatePoints(origin, scale);	
		//resectionCamera(2);
		saveSceneData(this->scene);
	}
	
	void Reconstructor::initializeTracks(Pair& viewPair)
	{
		this->tracksBuilder.Build(*matchesRCT);
		this->tracksBuilder.Filter();
		this->tracksBuilder.ExportToSTL(mapTracks);
		this->trackVisibility.reset(new openMVG::tracks::SharedTrackVisibilityHelper(mapTracks));
		this->trackVisibility->GetTracksInImages({ viewPair.first, viewPair.second }, mapTracksCommon);
	}

	void Reconstructor::initializeScene(int w, int h)
	{
		this->scene->intrinsics[0].reset(new cameras::Pinhole_Intrinsic(w, h, (*K)(0, 0), (*K)(0, 2), (*K)(1, 2)));

		this->camCurrent = dynamic_cast<cameras::Pinhole_Intrinsic*> (scene->intrinsics[0].get());
		this->cam_J = dynamic_cast<cameras::Pinhole_Intrinsic*> (scene->intrinsics[0].get());
	}
	
	void Reconstructor::triangulatePoints(Pose3 &origin, float &scale)
	{
		const size_t n = mapTracksCommon.size();
		Mat xI(2, n), xJ(2, n);
		uint32_t ptCurrIndex = 0;
		Vec2 feat;
		for (openMVG::tracks::STLMAPTracks::const_iterator iterT = mapTracksCommon.begin(); iterT != mapTracksCommon.end(); ++iterT, ++ptCurrIndex) {
			tracks::submapTrack::const_iterator iter = iterT->second.begin();
			const uint32_t i = iter->second;
			const uint32_t j = (++iter)->second;

			feat = regionsRCT->at(0)->GetRegionsPositions()[i].coords().cast<double>();
			xI.col(ptCurrIndex) = camCurrent->get_ud_pixel(feat);
			feat = regionsRCT->at(1)->GetRegionsPositions()[j].coords().cast<double>();
			xJ.col(ptCurrIndex) = cam_J->get_ud_pixel(feat);
		}

		Landmarks & landmarks = scene->structure;

		for (openMVG::tracks::STLMAPTracks::const_iterator
			iterT = mapTracksCommon.begin();
			iterT != mapTracksCommon.end();
			++iterT)
		{
			// Get corresponding points
			tracks::submapTrack::const_iterator iter = iterT->second.begin();
			const uint32_t i = iter->second;
			const uint32_t j = (++iter)->second;

			const Vec2 x1_ = regionsRCT->at(0)->GetRegionsPositions()[i].coords().cast<double>();
			const Vec2 x2_ = regionsRCT->at(1)->GetRegionsPositions()[j].coords().cast<double>();

			Vec3 X;

			Pose3 Pose_I = scene->poses[scene->views[0]->id_pose] = origin;

			Mat3 Rrelative = relativePosesRCT->at({ 0,1 }).relativePose.rotation();
			Vec3 Trelative = relativePosesRCT->at({ 0,1 }).relativePose.translation();
			Pose3 Pose_J = scene->poses[scene->views[1]->id_pose] = relativePoseToAbsolute(origin, Pose3(Rrelative, scale*Trelative));

			Mat34 P1, P2;
			constructProjectionsTriangulation(Pose_I, Pose_J, P1, P2);
			TriangulateDLT(P1, x1_.homogeneous(), P2, x2_.homogeneous(), &X);
			Observations obs;
			obs[scene->views[0]->id_view] = Observation(x1_, i);
			obs[scene->views[1]->id_view] = Observation(x2_, j);
			landmarks[iterT->first].obs = std::move(obs);
			landmarks[iterT->first].X = X;
		}
	}

	void Reconstructor::constructProjectionsTriangulation(Pose3 &pose1, Pose3 &pose2, Mat34& P1, Mat34& P2)
	{
		P1 = camCurrent->get_projective_equivalent(pose1);
		P2 = cam_J->get_projective_equivalent(pose2);
	}

	Pose3 Reconstructor::relativePoseToAbsolute(Pose3 &relativePose, Pose3 &origin)
	{
		Mat3 R1 = origin.rotation();
		Mat3 R2 = relativePose.rotation();

		Mat3 Rfinal = R2*R1;

		Vec3 t1 = origin.translation();
		Vec3 t2 = relativePose.translation();

		Vec3 tfinal = t1 + t2;

		return Pose3(Rfinal, tfinal);
	}
	
	bool Reconstructor::resectionCamera(unsigned int id)
	{
		openMVG::tracks::STLMAPTracks newCommonTracks;
		trackVisibility->GetTracksInImages({ id }, newCommonTracks);
		std::set<uint32_t> set_tracksIds;
		openMVG::tracks::TracksUtilsMap::GetTracksIdVector(newCommonTracks, &set_tracksIds);

		std::set<uint32_t> mapTrackId;
		std::transform(scene->GetLandmarks().begin(), scene->GetLandmarks().end(), std::inserter(mapTrackId, mapTrackId.begin()), stl::RetrieveKey());

		std::set<uint32_t> resectionTrackId;
		std::set_intersection(set_tracksIds.cbegin(), set_tracksIds.cend(), mapTrackId.cbegin(), mapTrackId.cend(), std::inserter(resectionTrackId, resectionTrackId.begin()));

		const PointFeatures resectionFeats = regionsRCT->at(id)->GetRegionsPositions();

		if (resectionTrackId.empty()) {
			// No match. The image has no connection with already reconstructed points.
			std::cout << std::endl
				<< "-------------------------------" << "\n"
				<< "-- Resection of camera index: " << "\n"
				<< "-- Resection status: " << "FAILED" << "\n"
				<< "-------------------------------" << std::endl;
			return false;
		}

		std::vector<uint32_t> resectionFeatId;
		openMVG::tracks::TracksUtilsMap::GetFeatIndexPerViewAndTrackId(newCommonTracks, resectionTrackId, id, &resectionFeatId);

		Image_Localizer_Match_Data resectionData;
		resectionData.pt2D.resize(2, resectionTrackId.size());
		resectionData.pt3D.resize(3, resectionTrackId.size());		

		const View * viewCurrent = scene->GetViews().at(id).get();
		std::shared_ptr<cameras::IntrinsicBase> intrinsicCurr(nullptr);
		if (scene->GetIntrinsics().count(viewCurrent->id_intrinsic))
			intrinsicCurr = scene->GetIntrinsics().at(viewCurrent->id_intrinsic);

		// Setup the track 2d observation for this new view
		Mat2X pt2DOriginal(2, resectionTrackId.size());
		std::set<uint32_t>::const_iterator iterTrackId = resectionTrackId.begin();
		std::vector<uint32_t>::const_iterator iterfeatId = resectionFeatId.begin();
		for (size_t ptCurr = 0; ptCurr < resectionFeatId.size(); ++ptCurr, ++iterTrackId, ++iterfeatId)
		{
			resectionData.pt3D.col(ptCurr) = scene->GetLandmarks().at(*iterTrackId).X;
			resectionData.pt2D.col(ptCurr) = pt2DOriginal.col(ptCurr) = resectionFeats[*iterfeatId].coords().cast<double>();
			// Handle image distortion if intrinsic is known (to ease the resection)
			if (intrinsicCurr && intrinsicCurr->have_disto())
			{
				resectionData.pt2D.col(ptCurr) = intrinsicCurr->get_ud_pixel(resectionData.pt2D.col(ptCurr));
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
			{ viewCurrent->ui_width, viewCurrent->ui_height },
			intrinsicCurr.get(),
			resectionData,
			pose
		);
		resectionData.pt2D = std::move(pt2DOriginal); // restore original image domain points


		std::cout << std::endl
			<< "-------------------------------" << "<br>"
			<< "-- Robust Resection of camera index: <" << "> image: "
			<< viewCurrent->s_Img_path << "<br>"
			<< "-- Threshold: " << resectionData.error_max << "<br>"
			<< "-- Resection status: " << (bResection ? "OK" : "FAILED") << "<br>"
			<< "-- Nb points used for Resection: " << resectionFeatId.size() << "<br>"
			<< "-- Nb points validated by robust estimation: " << resectionData.vec_inliers.size() << "<br>"
			<< "-- % points validated: "
			<< resectionData.vec_inliers.size() / static_cast<float>(resectionFeatId.size()) << "<br>"
			<< "-------------------------------" << "<br>";


		scene->poses[viewCurrent->id_pose] = pose;

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
		scene->intrinsics[new_intrinsic_id] = intrinsicCurr;


		// F. List tracks that share content with this view and add observations and new 3D track if required.
		//    - If the track already exists (look if the new view tracks observation are valid)
		//    - If the track does not exists, try robust triangulation & add the new valid view track observation
		{
			// Get information of new view
			const IndexT I = id;
			const View * viewCurrent = scene->GetViews().at(I).get();
			const cameras::IntrinsicBase * camCurrent = scene->GetIntrinsics().at(viewCurrent->id_intrinsic).get();
			const Pose3 pose_I = scene->GetPoseOrDie(viewCurrent);

			// Vector of all already reconstructed views
			const std::set<IndexT> validViews = Get_Valid_Views(*scene);

			// Go through each track and look if we must add new view observations or new 3D points
			for (const std::pair< uint32_t, tracks::submapTrack >& trackIt : newCommonTracks)
			{
				const uint32_t trackId = trackIt.first;
				const tracks::submapTrack & track = trackIt.second;
				// List the potential view observations of the track
				const tracks::submapTrack & trackViews = mapTracks[trackId];

				// List to save the new view observations that must be added to the track
				std::set<IndexT> newTracks;
				// If the track was already reconstructed
				if (scene->structure.count(trackId) != 0)
				{
					// Since the 3D point was triangulated before we add the new the Inth view observation
					newTracks.insert(I);
				}
				else
				{
					// Go through the views that observe this track & look if a successful triangulation can be done
					for (const std::pair< IndexT, IndexT >& trackViewIt : trackViews)
					{
						const IndexT & J = trackViewIt.first;
						// If view is valid try triangulation
						if (J != I && validViews.count(J) != 0)
						{
							// If successfuly triangulated add the observation from J view
							if (scene->structure.count(trackId) != 0)
							{
								newTracks.insert(J);
							}
							else
							{
								const View * view_J = scene->GetViews().at(J).get();
								const cameras::IntrinsicBase * cam_J = scene->GetIntrinsics().at(view_J->id_intrinsic).get();
								const Pose3 pose_J = scene->GetPoseOrDie(view_J);
								const Vec2 xJ = regionsRCT->at(J)->GetRegionsPositions()[trackViews.at(J)].coords().cast<double>();

								// Position of the point in view I
								const Vec2 xI = regionsRCT->at(I)->GetRegionsPositions()[track.at(I)].coords().cast<double>();

								// Try to triangulate a 3D point from J view
								// A new 3D point must be added
								// Triangulate it
								const Vec2 xI_ud = camCurrent->get_ud_pixel(xI);
								const Vec2 xJ_ud = cam_J->get_ud_pixel(xJ);
								const Mat34 P_I = camCurrent->get_projective_equivalent(pose_I);
								const Mat34 P_J = cam_J->get_projective_equivalent(pose_J);
								Vec3 X = Vec3::Zero();
								TriangulateDLT(P_I, xI_ud.homogeneous(), P_J, xJ_ud.homogeneous(), &X);
								// Check triangulation result
								const double angle = AngleBetweenRay(pose_I, camCurrent, pose_J, cam_J, xI, xJ);
								const Vec2 residual_I = camCurrent->residual(pose_I(X), xI);
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
									newTracks.insert(I);
									newTracks.insert(J);
								} // 3D point is valid
								else
								{
									// We mark the view to add the observations once the point is triangulated
									newTracks.insert(J);
								} // 3D point is invalid
							}
						}
					}// Go through all the views
				}// If new point

				 // If successfuly triangulated, add the valid view observations
				if (scene->structure.count(trackId) != 0 && !newTracks.empty())
				{
					Landmark & landmark = scene->structure[trackId];
					// Check if view feature point observations of the track are valid (residual, depth) or not
					for (const IndexT &J : newTracks)
					{
						const View * view_J = scene->GetViews().at(J).get();
						const cameras::IntrinsicBase * cam_J = scene->GetIntrinsics().at(view_J->id_intrinsic).get();
						const Pose3 pose_J = scene->GetPoseOrDie(view_J);
						const Vec2 xJ = regionsRCT->at(J)->GetRegionsPositions()[trackViews.at(J)].coords().cast<double>();

						const Vec2 residual = cam_J->residual(pose_J(landmark.X), xJ);
						if (pose_J.depth(landmark.X) > 0)
						{
							landmark.obs[J] = Observation(xJ, trackViews.at(J));
						}
					}
				}
			}// All the tracks in the view
		}
	}

	bool Reconstructor::saveSceneData(SfM_Data* scene)
	{
		if (openMVG::sfm::Save(*scene, stlplus::create_filespec(*currentFolder, "map.ply"), ESfM_Data(ALL)))
			return 1;
		else
			return 0;
	}
}