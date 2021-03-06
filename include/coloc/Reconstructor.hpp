//
// Created by sai on 7/10/18.
//
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

#include <cmath>

#include "colocData.hpp"
#include "Refiner.hpp"

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::sfm;
using namespace openMVG::geometry;

namespace coloc
{
    class Reconstructor {
    public:
        explicit Reconstructor(colocParams& params)
                : imageSize(&params.imageSize), K(&params.K), dist(&params.dist),
                  currentFolder(&params.imageFolder)
        {	}

        void reconstructScene(bool inter, colocData& data, std::vector <Pose3>, float, bool);
		void Reconstructor::interReconstruct(int sourceId, int destId, colocData& data);

    private:
        // Internal methods

        void initializeTracks(Pair& viewPair);
        void initializeScene(int w, int h, Pair& viewPair);
        void triangulatePoints(Pair& pair, Pose3&, float&);
        void constructProjectionsTriangulation(Pose3&, Pose3&, Mat34& P1, Mat34& P2);
        bool resectionCamera(unsigned int);
        bool saveSceneData(Scene* scene, std::string& fileName);
        Pose3 relativePoseToAbsolute(Pose3&, Pose3&);

        // Internal variables

        tracks::TracksBuilder tracksBuilder;
        openMVG::tracks::STLMAPTracks mapTracks, mapTracksCommon;
        std::unique_ptr<openMVG::tracks::SharedTrackVisibilityHelper> trackVisibility;
        cameras::Pinhole_Intrinsic *camCurrent, *cam_J;
        std::pair <int, int> *imageSize;
        std::vector <Mat3> *K;
		std::vector <Vec3> *dist;
        std::string *currentFolder;
        PoseRefiner refiner;

        // Local pointers to external localization data

        FeatureMap * regionsRCT;
        PairWiseMatches* matchesRCT;
        std::map<Pair, RelativePose_Info > * relativePosesRCT;
        Scene* scene;
    };

	void Reconstructor::interReconstruct(int sourceId, int destId, colocData& data)
	{
		this->regionsRCT = &data.regions;
		this->matchesRCT = &data.geometricMatches;
		this->relativePosesRCT = &data.relativePoses;
		this->scene = &data.tempScene;

		float scale = 1.0;
		Pair seedPair = std::make_pair(sourceId, destId);

		std::cout << "Using pair " << seedPair.first << " & " << seedPair.second << "as seed" << std::endl;

		data.keyframeIdx = seedPair.first;
		initializeTracks(seedPair);
		initializeScene(imageSize->first, imageSize->second, seedPair);

		std::cout << "Triangulating feature matches... ";
        Pose3 tempOrigin = Pose3(Mat3::Identity(), Vec3::Zero());
		triangulatePoints(seedPair, tempOrigin, scale);
		std::cout << "Done." << std::endl;
	}

    void Reconstructor::reconstructScene(bool inter, colocData& data, std::vector <Pose3> poses, float scale = 1.0, bool Adjust = false)
    {
        this->regionsRCT = &data.regions;
        this->matchesRCT = &data.geometricMatches;
        this->relativePosesRCT = &data.relativePoses;
		Pose3 origin;

		int maxMatches = 0;
		Pair seedPair;

		for (auto matchedPair : *matchesRCT) {
			Pair currentPair = matchedPair.first;
			if (matchedPair.second.size() > maxMatches) {
				maxMatches = matchedPair.second.size();
				seedPair = currentPair;
			}
		}

		if (inter) {
			this->scene = &data.tempScene;
			origin = Pose3(Mat3::Identity(), Vec3::Zero());
		}
		else {
			this->scene = &data.scene;
			origin = poses[seedPair.first];
		}
		
		std::cout << "Using pair " << seedPair.first << " & " << seedPair.second << "as seed" << std::endl;

		data.keyframeIdx = seedPair.first;

		std::vector<unsigned int> resectionList;

		for (unsigned int i = 0; i < regionsRCT->size(); ++i)
		{
			if (i != seedPair.first && i != seedPair.second)
				resectionList.push_back(i);
		}

        initializeTracks(seedPair);
        initializeScene(imageSize->first, imageSize->second, seedPair);

        std::cout << "Triangulating feature matches... ";
        triangulatePoints(seedPair, origin, scale);
        std::cout << "Done." << std::endl;

		std::string initialMap = "initial.ply";
		saveSceneData(this->scene, initialMap);
        const Optimize_Options ba_refine_options
                (cameras::Intrinsic_Parameter_Type::NONE, Extrinsic_Parameter_Type::ADJUST_ALL, Structure_Parameter_Type::ADJUST_ALL);
       
		for (auto resectionId : resectionList)
			resectionCamera(resectionId);

		std::string refinedMap = "refined.ply";
        std::cout << "Refining scene...";
        float rmse;
        Cov6 cov;
        if(Adjust)
            refiner.refinePose(*scene, ba_refine_options, rmse, cov);
        std::cout << "Done." << std::endl;
        saveSceneData(this->scene, refinedMap);
    }

    void Reconstructor::initializeTracks(Pair& viewPair)
    {
        this->tracksBuilder.Build(*matchesRCT);
        this->tracksBuilder.Filter();
        this->tracksBuilder.ExportToSTL(mapTracks);
        this->trackVisibility.reset(new openMVG::tracks::SharedTrackVisibilityHelper(mapTracks));
        this->trackVisibility->GetTracksInImages({ viewPair.first, viewPair.second }, mapTracksCommon);
    }

    void Reconstructor::initializeScene(int w, int h, Pair& seedPair)
    {
        for (int camId = 0; camId < K->size(); camId++)
			this->scene->intrinsics[camId].reset(new cameras::Pinhole_Intrinsic_Radial_K3(w, h, (*K)[camId](0, 0), (*K)[camId](0, 2), (*K)[camId](1, 2), 
																								(*dist)[camId](0), (*dist)[camId](1), (*dist)[camId](2)));

        this->camCurrent = dynamic_cast<cameras::Pinhole_Intrinsic*> (scene->intrinsics[seedPair.first].get());
        this->cam_J = dynamic_cast<cameras::Pinhole_Intrinsic*> (scene->intrinsics[seedPair.second].get());
    }

    void Reconstructor::triangulatePoints(Pair& pair, Pose3 &origin, float &scale)
    {
        const size_t n = mapTracksCommon.size();
        Mat xI(2, n), xJ(2, n);
        uint32_t ptCurrIndex = 0;
        Vec2 feat;
        for (openMVG::tracks::STLMAPTracks::const_iterator iterT = mapTracksCommon.begin(); iterT != mapTracksCommon.end(); ++iterT, ++ptCurrIndex) {
            tracks::submapTrack::const_iterator iter = iterT->second.begin();
            const uint32_t i = iter->second;
            const uint32_t j = (++iter)->second;

            feat = regionsRCT->at(pair.first)->GetRegionsPositions()[i].coords().cast<double>();
            xI.col(ptCurrIndex) = camCurrent->get_ud_pixel(feat);
            feat = regionsRCT->at(pair.second)->GetRegionsPositions()[j].coords().cast<double>();
            xJ.col(ptCurrIndex) = cam_J->get_ud_pixel(feat);
        }

        Landmarks & landmarks = scene->structure;

        for (openMVG::tracks::STLMAPTracks::const_iterator iterT = mapTracksCommon.begin(); iterT != mapTracksCommon.end();	++iterT) {
            // Get corresponding points
            tracks::submapTrack::const_iterator iter = iterT->second.begin();
            const uint32_t i = iter->second;
            const uint32_t j = (++iter)->second;

            const Vec2 x1_ = camCurrent->get_ud_pixel(regionsRCT->at(pair.first)->GetRegionsPositions()[i].coords().cast<double>());
            const Vec2 x2_ = cam_J->get_ud_pixel(regionsRCT->at(pair.second)->GetRegionsPositions()[j].coords().cast<double>());

            Vec3 X;

            Pose3 Pose_I = scene->poses[scene->views[pair.first]->id_pose] = origin;

            Mat3 Rrelative = relativePosesRCT->at(pair).relativePose.rotation();
            Vec3 Trelative = relativePosesRCT->at(pair).relativePose.center();
            Pose3 relativePose = Pose3(Rrelative, scale*Trelative);

            Pose3 Pose_J = scene->poses[scene->views[pair.second]->id_pose] = relativePoseToAbsolute(origin, relativePose);

            Mat34 P1, P2;
            constructProjectionsTriangulation(Pose_I, Pose_J, P1, P2);
            TriangulateDLT(P1, x1_.homogeneous(), P2, x2_.homogeneous(), &X);

			if (Pose_I.depth(X) < 0 && Pose_J.depth(X) < 0)
				continue;

			if (std::abs(X[2]) > 100.0)
				continue;

            Observations obs;
            obs[scene->views[pair.first]->id_view] = Observation(x1_, i);
            obs[scene->views[pair.second]->id_view] = Observation(x2_, j);
            landmarks[iterT->first].obs = std::move(obs);
            landmarks[iterT->first].X = X;
        }
    }

    void Reconstructor::constructProjectionsTriangulation(Pose3 &pose1, Pose3 &pose2, Mat34& P1, Mat34& P2)
    {
        P1 = camCurrent->get_projective_equivalent(pose1);
        P2 = cam_J->get_projective_equivalent(pose2);
    }

    Pose3 Reconstructor::relativePoseToAbsolute(Pose3 &origin, Pose3 &relativePose)
    {
        Mat3 R1 = origin.rotation();
        Mat3 R2 = relativePose.rotation();
        Mat3 Rfinal = R2*R1;
        Vec3 t1 = origin.center();
        Vec3 t2 = relativePose.center();
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
            std::cout << "Resection of view " << id << " failed." << std::endl;
            return EXIT_FAILURE;
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
            if (intrinsicCurr && intrinsicCurr->have_disto()) {
                resectionData.pt2D.col(ptCurr) = intrinsicCurr->get_ud_pixel(resectionData.pt2D.col(ptCurr));
            }
        }

        geometry::Pose3 pose;
        const bool bResection = sfm::SfM_Localizer::Localize(resection::SolverType::P3P_KE_CVPR17, { viewCurrent->ui_width, viewCurrent->ui_height },
                                                             intrinsicCurr.get(), resectionData, pose);
        resectionData.pt2D = std::move(pt2DOriginal);

        std::cout << "Robust Resection of view " << id << " :" << std::endl
                  << "Threshold: " << resectionData.error_max << std::endl
                  << "Resection status: " << (bResection ? "Success" : "Failure") << std::endl
                  << "Feature points tracked: " << resectionFeatId.size() << std::endl
                  << "Feature points validated: " << resectionData.vec_inliers.size() << std::endl;

        scene->poses[viewCurrent->id_pose] = pose;

        IndexT new_intrinsic_id = 0;
        if (!scene->GetIntrinsics().empty()) {
            std::set<IndexT> existing_intrinsicId;
            std::transform(scene->GetIntrinsics().begin(), scene->GetIntrinsics().end(),
                           std::inserter(existing_intrinsicId, existing_intrinsicId.begin()),
                           stl::RetrieveKey());
            new_intrinsic_id = (*existing_intrinsicId.rbegin()) + 1;
        }
        scene->views.at(id)->id_intrinsic = new_intrinsic_id;
        scene->intrinsics[new_intrinsic_id] = intrinsicCurr;

        {
            const IndexT I = id;
            const View * viewCurrent = scene->GetViews().at(I).get();
            const cameras::IntrinsicBase * camCurrent = scene->GetIntrinsics().at(viewCurrent->id_intrinsic).get();
            const Pose3 pose_I = scene->GetPoseOrDie(viewCurrent);


            const std::set<IndexT> validViews = Get_Valid_Views(*scene);

            // Go through each track and look if we must add new view observations or new 3D points
            for (const std::pair< uint32_t, tracks::submapTrack >& trackIt : newCommonTracks) {
                const uint32_t trackId = trackIt.first;
                const tracks::submapTrack & track = trackIt.second;
                // List the potential view observations of the track
                const tracks::submapTrack & trackViews = mapTracks[trackId];

                // List to save the new view observations that must be added to the track
                std::set<IndexT> newTracks;
                // If the track was already reconstructed
                if (scene->structure.count(trackId) != 0) {
                    // Since the 3D point was triangulated before we add the new the Inth view observation
                    newTracks.insert(I);
                }
                else {
                    // Go through the views that observe this track & look if a successful triangulation can be done
                    for (const std::pair< IndexT, IndexT >& trackViewIt : trackViews) {
                        const IndexT & J = trackViewIt.first;
                        // If view is valid try triangulation
                        if (J != I && validViews.count(J) != 0) {
                            // If successfuly triangulated add the observation from J view
                            if (scene->structure.count(trackId) != 0)
                                newTracks.insert(J);
                            else {
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
                                if (angle > 2.0 && pose_I.depth(X) > 0 && pose_J.depth(X) > 0 && std::abs(X[2]) < 1000.0) {
                                    // Add a new track
                                    Landmark & landmark = scene->structure[trackId];
                                    landmark.X = X;
                                    newTracks.insert(I);
                                    newTracks.insert(J);
                                } // 3D point is valid
                                else
                                    newTracks.insert(J);

                            }
                        }
                    }// Go through all the views
                }// If new point

                // If successfuly triangulated, add the valid view observations
                if (scene->structure.count(trackId) != 0 && !newTracks.empty()) {
                    Landmark & landmark = scene->structure[trackId];
                    // Check if view feature point observations of the track are valid (residual, depth) or not
                    for (const IndexT &J : newTracks) {
                        const View * view_J = scene->GetViews().at(J).get();
                        const cameras::IntrinsicBase * cam_J = scene->GetIntrinsics().at(view_J->id_intrinsic).get();
                        const Pose3 pose_J = scene->GetPoseOrDie(view_J);
                        const Vec2 xJ = regionsRCT->at(J)->GetRegionsPositions()[trackViews.at(J)].coords().cast<double>();
                        const Vec2 residual = cam_J->residual(pose_J(landmark.X), xJ);

                        if (pose_J.depth(landmark.X) > 0 && std::abs(landmark.X[2]) < 1000.0)
                            landmark.obs[J] = Observation(xJ, trackViews.at(J));
                    }
                }
            }
        }// All the tracks in the view
    }

    bool Reconstructor::saveSceneData(Scene* scene, std::string& fileName)
    {
        if (openMVG::sfm::Save(*scene, stlplus::create_filespec(*currentFolder, fileName), ESfM_Data(ALL)))
            return EXIT_SUCCESS;
        else
            return EXIT_FAILURE;
    }
}