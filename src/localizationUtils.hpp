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
		bool matchMaps(SfM_Data scene1, SfM_Data scene2);
		bool matchSceneWithMap(SfM_Data scene);
		bool setupMap(LocalizationData&, LocalizationParams&);
		bool initMapMatchingInterface(LocalizationData &data, Regions_Provider &mapFeatures);

	private:
		std::shared_ptr<Regions_Provider> mapFeatures = std::make_shared<Regions_Provider>();
	};

	bool Utils::setupMap(LocalizationData &data, LocalizationParams &params)
	{
		if (data.scene.GetPoses().empty() || data.scene.GetLandmarks().empty()) {
			std::cerr << "The input scene has no 3D content." << std::endl;
			return 1;
		}

		C_Progress_display progress;
		std::unique_ptr<Regions> regions_type;
		regions_type.reset(new features::AKAZE_Binary_Regions());

		mapFeatures->load(data.scene, params.imageFolder, regions_type, &progress);

		if (initMapMatchingInterface(data, *mapFeatures.get())) {
			std::cerr << "Cannot initialize the SfM localizer" << std::endl;
			return EXIT_FAILURE;
		}
		else
			std::cout << "Initialized SFM localizer with map" << std::endl;

		mapFeatures.reset();
		return EXIT_SUCCESS;
	}

	bool Utils::initMapMatchingInterface(LocalizationData &data, Regions_Provider& mapFeatures)
	{
		if (data.scene.GetPoses().empty() || data.scene.GetLandmarks().empty()) {
			std::cerr << std::endl << "The input SfM_Data file have not 3D content to match with." << std::endl;
			return EXIT_FAILURE;
		}

		data.mapRegions.reset(mapFeatures.getRegionsType()->EmptyClone());
		for (const auto & landmark : data.scene.GetLandmarks())
		{
			for (const auto & observation : landmark.second.obs)
			{
				if (observation.second.id_feat != UndefinedIndexT)
				{
					const std::shared_ptr<features::Regions> viewRegions = mapFeatures.get(observation.first);
					viewRegions->CopyRegion(observation.second.id_feat, data.mapRegions.get());
					data.mapRegionIdx.push_back(landmark.first);
				}
			}
		}
		return EXIT_SUCCESS;
	}
}
