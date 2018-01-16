#pragma once

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
#include "openMVG/matching/svg_matches.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include "localizationParams.hpp"
#include "localizationData.hpp"
#include "estimationKernel.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::sfm;

namespace coloc
{
	class Utils
	{
	public:
		void matchMaps(LocalizationParams& params, LocalizationData &data1, LocalizationData &data2, std::vector<IndMatch> &commonFeatures);
		bool computeScaleDifference(LocalizationParams& params, LocalizationData &data1, LocalizationData &data2, float &scaleDiff);
		bool matchSceneWithMap(Scene& scene);
		bool setupMap(LocalizationData&, LocalizationParams&);
		bool initMapMatchingInterface(LocalizationData &data, Regions_Provider &mapFeatures);
		bool rescaleMap(Scene& scene, float scale);
		int drawFeaturePoints(std::string& imageName, features::PointFeatures points);
		bool drawMatches(std::string& outputFilename, std::string& image1, std::string& image2, Regions& regions1, Regions& regions2, std::vector <IndMatch>& matches);
		void filterMapMatches(LocalizationParams& params, Regions& regions1, Regions& regions2, std::vector<IndMatch>& commonFeatures, std::vector<IndMatch>& filteredMatches);
	private:
		
		std::shared_ptr<Regions_Provider> mapFeatures;
		EMatcherType matchingType;
	};

	bool Utils::drawMatches(std::string& outputFilename, std::string& image1, std::string& image2, Regions& regions1, Regions& regions2, std::vector <IndMatch>& matches)
	{
		Matches2SVG
		(
			image1,
			{ 640, 480 },
			regions1.GetRegionsPositions(),
			image2,
			{ 640, 480 },
			regions2.GetRegionsPositions(),
			matches,
			outputFilename,
			true
		);
		return Success;
	}

	bool Utils::setupMap(LocalizationData& data, LocalizationParams& params)
	{
		mapFeatures = std::make_shared<Regions_Provider>();
		
		if (data.scene.GetPoses().empty() || data.scene.GetLandmarks().empty()) {
			std::cerr << "The input scene has no 3D content." << std::endl;
			return Failure;
		}

		C_Progress_display progress;
		std::unique_ptr<Regions> regions_type;

		if (params.featureDetectorType == "AKAZE") {
			regions_type.reset(new openMVG::features::AKAZE_Float_Regions);
			matchingType = BRUTE_FORCE_L2;
		}
		else if (params.featureDetectorType == "SIFT") {
			regions_type.reset(new openMVG::features::SIFT_Regions);
			matchingType = CASCADE_HASHING_L2;
		}
		else if (params.featureDetectorType == "BINARY") {
			regions_type.reset(new openMVG::features::AKAZE_Binary_Regions);
			matchingType = BRUTE_FORCE_HAMMING;
		}

		mapFeatures->load(data.scene, params.imageFolder, regions_type, &progress);

		if (!initMapMatchingInterface(data, *mapFeatures.get())) {
			std::cerr << "Cannot initialize the SfM localizer" << std::endl;
			return Failure;
		}
		else
			std::cout << "Initialized SFM localizer with map" << std::endl;

		mapFeatures.reset();
		return Success;
	}

	bool Utils::initMapMatchingInterface(LocalizationData &data, Regions_Provider& mapFeatures)
	{
		if (data.scene.GetPoses().empty() || data.scene.GetLandmarks().empty()) {
			std::cerr << std::endl << "The input Scene file have not 3D content to match with." << std::endl;
			return Failure;
		}

		data.mapRegions.reset(mapFeatures.getRegionsType()->EmptyClone());
		for (const auto & landmark : data.scene.GetLandmarks()) {
			if (landmark.second.obs.at(0).id_feat != UndefinedIndexT) {
				const std::shared_ptr<features::Regions> viewRegions = mapFeatures.get(0);
				viewRegions->CopyRegion(landmark.second.obs.at(0).id_feat, data.mapRegions.get());
				data.mapRegionIdx.push_back(landmark.first);
			}
		}
		return Success;
	}

	void Utils::filterMapMatches(LocalizationParams& params, Regions& regions1, Regions& regions2, std::vector<IndMatch>& commonFeatures, std::vector<IndMatch>& filteredMatches)
	{
		const PointFeatures featI = regions1.GetRegionsPositions();
		const PointFeatures featJ = regions2.GetRegionsPositions();

		Mat xL(2, commonFeatures.size());
		Mat xR(2, commonFeatures.size());
		for (size_t k = 0; k < commonFeatures.size(); ++k) {
			xL.col(k) = featI[commonFeatures[k].i_].coords().cast<double>();
			xR.col(k) = featJ[commonFeatures[k].j_].coords().cast<double>();
		}

		RelativePose_Info relativePose;

		const Pinhole_Intrinsic
			camL(params.imageSize.first, params.imageSize.second, (params.K)(0, 0), (params.K)(0, 2), (params.K)(1, 2)),
			camR(params.imageSize.first, params.imageSize.second, (params.K)(0, 0), (params.K)(0, 2), (params.K)(1, 2));

		if (!robustRelativePose(&camL, &camR, xL, xR, relativePose, params.imageSize, params.imageSize, 256)) {
			std::cerr << " /!\\ Robust relative pose estimation failure." << std::endl;
		}

		else {
			std::cout << "Filtering complete, number of inliers found: " << relativePose.vec_inliers.size() << "\n"
				<< "\t Total number of matches: " << commonFeatures.size()
				<< std::endl;
		}
		for (int ic = 0; ic < relativePose.vec_inliers.size(); ++ic)
			filteredMatches.push_back(commonFeatures[relativePose.vec_inliers[ic]]);

		std::string file1 = params.imageFolder + "img__Quad0_0000.png";
		std::string file2 = params.imageFolder + "img__Quad2_0251.png";

		//drawMatches(file1, file2, regions1, regions2, filteredMatches);
	}

	void Utils::matchMaps(LocalizationParams& params, LocalizationData &data1, LocalizationData &data2, std::vector<IndMatch> &commonFeatures)
	{
		std::vector <IndMatch> putativeMatches, homofiltered;
		
		matching::DistanceRatioMatch(
			0.8, matchingType,
			*data1.mapRegions.get(),
			*data2.mapRegions.get(),
			putativeMatches);

		std::string file1 = params.imageFolder + "img__Quad0_0000.png";
		std::string file2 = params.imageFolder + "img__Quad2_0251.png";
		//drawMatches(file1, file2, *data1.mapRegions.get(), *data2.mapRegions.get(), putativeMatches);
		filterHomography(params, *data1.mapRegions.get(), *data2.mapRegions.get(), { 0,1 }, putativeMatches, homofiltered);
		filterMapMatches(params, *data1.mapRegions.get(), *data2.mapRegions.get(), homofiltered, commonFeatures);
	}

	bool Utils::computeScaleDifference(LocalizationParams& params, LocalizationData &data1, LocalizationData &data2, float &scaleDiff)
	{
		std::vector<IndMatch> commonFeatures;
		matchMaps(params, data1, data2, commonFeatures);

		if (commonFeatures.empty())
		{
			std::cout << "No common features between the maps." << std::endl;
			return EXIT_FAILURE;
		}

		std::cout << "Number of common features: " << commonFeatures.size() << std::endl;

		Mat pt3D_1, pt3D_2;
		
		pt3D_1.resize(3, commonFeatures.size());
		pt3D_2.resize(3, commonFeatures.size());

		float scale = 0.0;

		for (size_t i = 0; i < commonFeatures.size(); ++i)
		{
			pt3D_1.col(i) = data1.scene.GetLandmarks().at(data1.mapRegionIdx[commonFeatures[i].i_]).X;
			pt3D_2.col(i) = data2.scene.GetLandmarks().at(data2.mapRegionIdx[commonFeatures[i].j_]).X;	
		}

		for (size_t i = 0; i < commonFeatures.size() - 1; ++i)
		{
			Vec3 X11 = data1.scene.GetLandmarks().at(data1.mapRegionIdx[commonFeatures[i].i_]).X;
			Vec3 X12 = data1.scene.GetLandmarks().at(data1.mapRegionIdx[commonFeatures[i + 1].i_]).X;
			
			Vec3 X21 = data2.scene.GetLandmarks().at(data2.mapRegionIdx[commonFeatures[i].j_]).X;
			Vec3 X22 = data2.scene.GetLandmarks().at(data2.mapRegionIdx[commonFeatures[i + 1].j_]).X;

			float dist1 = (X12 - X11).norm();
			float dist2 = (X22 - X21).norm();

			scale += std::max(dist1, dist2) / std::min(dist1, dist2);
		}
		scaleDiff = scale / (commonFeatures.size() - 1);
	}

	bool Utils::rescaleMap(Scene& scene, float scale)
	{
		for (unsigned int i = 0; i < scene.structure.size(); ++i) {
			scene.structure[i].X = scene.structure.at(i).X * scale;
		}

		for (unsigned int i = 0; i < scene.poses.size(); ++i) {
			scene.poses[i] = Pose3(scene.poses.at(i).rotation(), scene.poses.at(i).translation() * scale);
		}
		return Success;
	}
}
