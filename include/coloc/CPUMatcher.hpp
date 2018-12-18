//
// CoLoC : Collaborative Localization

// CPUMatcher.hpp: CPU based feature matching using OpenMVG implementations
// Created by Sai Vemprala on 7/8/18.
//

#pragma once

#include "coloc/colocParams.hpp"
#include "coloc/colocData.hpp"
#include "coloc/FeatureMatcher.hpp"
#include "coloc/colocUtils.hpp"

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::sfm;

namespace coloc
{
	template <typename T>
	class CPUMatcher {

	private:
		std::unique_ptr<openMVG::features::Regions> regions_type;
		std::string featFile, descFile;
		PairWiseMatches putativeMatches;
		std::map<Pair, unsigned int> overlap;
		EMatcherType matchingType;

	public:
		CPUMatcher (MatcherOptions &opts)
		{	
			regions_type.reset(new openMVG::features::AKAZE_Binary_Regions);
			matchingType = BRUTE_FORCE_HAMMING;
		}

		T computeMatches(FeatureMap& regions, PairWiseMatches &putativeMatches)
		{
			Pair_Set pairs = Utils::handlePairs(static_cast<int> (regions.size()));

			IndMatches pairMatches;
			for (const auto pairIdx : pairs) {
				computeMatchesPair(pairIdx, regions, pairMatches);
				if (!pairMatches.empty()) {
					putativeMatches.insert({ pairIdx, std::move(pairMatches) });
					overlap.insert({ pairIdx, std::move(pairMatches.size()) });
				}
				else {
					overlap.insert({ pairIdx, 0 });
				}
			}
			return EXIT_SUCCESS;
		}

		bool matchMapFeatures(std::unique_ptr<features::AKAZE_Binary_Regions> &scene1, std::unique_ptr<features::AKAZE_Binary_Regions> &scene2, std::vector<IndMatch> &commonFeatures)
		{
			matching::DistanceRatioMatch(
				0.8, BRUTE_FORCE_HAMMING,
				*scene1.get(),
				*scene2.get(),
				commonFeatures);

			return EXIT_SUCCESS;
		}

		bool computeMatchesPair(const Pair& pairIdx, FeatureMap& regions, IndMatches& putativeMatches, float distRatio = 0.8f) {
			/*
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
			*/
			//std::vector<IndMatch> vec_PutativeMatches;

			matching::DistanceRatioMatch(
				distRatio, this->matchingType,
				*regions.at(pairIdx.first).get(),
				*regions.at(pairIdx.second).get(),
				putativeMatches);

			return EXIT_SUCCESS;
		}

		bool matchSceneWithMap(unsigned int idx, colocData &data, IndMatches &trackedFeatures)
		{
			if (&data.scene == nullptr) {
				std::cout << "No existing map. Localization cannot continue." << std::endl;
				return EXIT_FAILURE;
			}

			matching::DistanceRatioMatch(
				0.8, this->matchingType,
				*data.mapRegions.get(),
				*data.regions.at(idx),
				trackedFeatures);

			if (trackedFeatures.empty()) {
				std::cout << "Unable to track any features" << std::endl;
				return EXIT_FAILURE;
			}

			std::cout << "Number of tracked features: " << trackedFeatures.size() << std::endl;
			return EXIT_SUCCESS;
		}

		void setMapData(int kpMapNum, void* desc)
		{ }
	};
}