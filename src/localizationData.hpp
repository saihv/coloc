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

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "localizationParams.hpp"

using namespace openMVG;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::sfm;


namespace coloc
{
	class LocalizationData {
	public:
		LocalizationData& operator = (LocalizationData& data)
		{
			// regions = data.regions;
			this->putativeMatches = data.putativeMatches;
			this->geometricMatches = data.geometricMatches;
			this->relativePoses = data.relativePoses;
			this->overlap = data.overlap;
			this->scene = data.scene;

			for (auto const& x : data.regions)
				// this->regions.insert({ x.first, std::make_unique<Regions>(*x.second) });
				this->regions.emplace_hint(this->regions.end(), x.first, std::make_unique<Regions>(*x.second));
			
			return *this;
		}

		FeatureMap regions;
		PairWiseMatches putativeMatches, geometricMatches;
		InterPoseMap relativePoses;
		std::map<Pair, double> overlap;
		Scene scene;
		std::unique_ptr<features::Regions> mapRegions;
		std::vector<IndexT> mapRegionIdx;

		bool setupFeatureDatabase(LocalizationParams& params)
		{
			std::shared_ptr<Regions_Provider> mapFeatures;
			mapFeatures = std::make_shared<Regions_Provider>();

			if (scene.GetPoses().empty() || scene.GetLandmarks().empty()) {
				std::cerr << "The input scene has no 3D content." << std::endl;
				return Failure;
			}

			C_Progress_display progress;
			std::unique_ptr<Regions> regions_type;
			EMatcherType matchingType;

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

			mapFeatures->load(scene, params.imageFolder, regions_type, &progress);

			if (!initMapMatchingInterface(*mapFeatures.get())) {
				std::cerr << "Cannot initialize the SfM localizer" << std::endl;
				return Failure;
			}
			else
				std::cout << "Initialized SFM localizer with map" << std::endl;

			mapFeatures.reset();
			return Success;
		}

		bool initMapMatchingInterface(Regions_Provider& mapFeatures)
		{
			if (scene.GetPoses().empty() || scene.GetLandmarks().empty()) {
				std::cerr << std::endl << "The input Scene file have not 3D content to match with." << std::endl;
				return Failure;
			}

			mapRegions.reset(mapFeatures.getRegionsType()->EmptyClone());
			for (const auto & landmark : scene.GetLandmarks()) {
				if (landmark.second.obs.at(0).id_feat != UndefinedIndexT) {
					const std::shared_ptr<features::Regions> viewRegions = mapFeatures.get(0);
					viewRegions->CopyRegion(landmark.second.obs.at(0).id_feat, mapRegions.get());
					mapRegionIdx.push_back(landmark.first);
				}
			}
			return Success;
		}
	};
}