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
		float computeScaleDifference(LocalizationParams& params, LocalizationData &data1, LocalizationData &data2, std::vector<IndMatch> commonFeatures);
		bool matchSceneWithMap(Scene& scene);
		bool setupMap(LocalizationData&, LocalizationParams&);
		bool initMapMatchingInterface(LocalizationData &data, Regions_Provider &mapFeatures);
		bool rescaleMap(Scene& scene, float scale);

		int drawFeaturePoints(std::string& imageName, features::PointFeatures points);
		bool drawMatches(std::string& outputFilename, std::string& image1, std::string& image2, Regions& regions1, Regions& regions2, std::vector <IndMatch>& matches);
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
			false
		);
		return Success;
	}

	float Utils::computeScaleDifference(LocalizationParams& params, LocalizationData &data1, LocalizationData &data2, std::vector<IndMatch> commonFeatures)
	{
		if (commonFeatures.empty()) {
			std::cout << "No common features between the maps." << std::endl;
			return Failure;
		}

		std::cout << "Number of common features between the maps: " << commonFeatures.size() << std::endl;

		Mat pt3D_1, pt3D_2;
		
		pt3D_1.resize(3, commonFeatures.size());
		pt3D_2.resize(3, commonFeatures.size());

		float scale = 0.0, scaleDiff = 1.0;

		for (size_t i = 0; i < commonFeatures.size(); ++i) {
			pt3D_1.col(i) = data1.scene.GetLandmarks().at(data1.mapRegionIdx[commonFeatures[i].i_]).X;
			pt3D_2.col(i) = data2.scene.GetLandmarks().at(data2.mapRegionIdx[commonFeatures[i].j_]).X;	
		}

		for (size_t i = 0; i < commonFeatures.size() - 1; ++i) {
			Vec3 X11 = data1.scene.GetLandmarks().at(data1.mapRegionIdx[commonFeatures[i].i_]).X;
			Vec3 X12 = data1.scene.GetLandmarks().at(data1.mapRegionIdx[commonFeatures[i + 1].i_]).X;
			
			Vec3 X21 = data2.scene.GetLandmarks().at(data2.mapRegionIdx[commonFeatures[i].j_]).X;
			Vec3 X22 = data2.scene.GetLandmarks().at(data2.mapRegionIdx[commonFeatures[i + 1].j_]).X;

			float dist1 = (X12 - X11).norm();
			float dist2 = (X22 - X21).norm();

			scale += dist1/dist2;
		}
		scaleDiff = scale / (commonFeatures.size() - 1);
		return scaleDiff;
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
