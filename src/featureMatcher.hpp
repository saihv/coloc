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
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/matching/regions_matcher.hpp"

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::sfm;

namespace coloc
{
	class FeatureMatcher {
	public:
		FeatureMatcher(LocalizationParams& params);

		Pair_Set handlePairs(int);
		void computePairMatches(const Pair& pairIdx, std::unique_ptr<features::Regions>& regions1, std::unique_ptr<features::Regions>& regions2, PairWiseMatches& putativeMatches);
		void computeMatches(FeatureMap& regions, PairWiseMatches& putativeMatches);

	private:
		std::unique_ptr<openMVG::features::Regions> regions_type;
		std::string featFile, descFile;
		PairWiseMatches putativeMatches;
		std::map<Pair, unsigned int> overlap;
		EMatcherType matchingType;
	};

	FeatureMatcher::FeatureMatcher(LocalizationParams& params) 
	{
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
	}

	Pair_Set FeatureMatcher::handlePairs(int numImages)
	{
		return exhaustivePairs(numImages);
	}

	void FeatureMatcher::computeMatches(FeatureMap &regions, PairWiseMatches &putativeMatches)
	{
		int numImages = regions.size();
		Pair_Set pairs = handlePairs(numImages);

		for (const auto pairIdx : pairs)
		{
			computePairMatches(pairIdx, regions.at(pairIdx.first), regions.at(pairIdx.second), putativeMatches);
		}
	}

	void FeatureMatcher::computePairMatches(const Pair& pairIdx, std::unique_ptr<features::Regions>& regions1, std::unique_ptr<features::Regions>& regions2, PairWiseMatches& putativeMatches)
	{
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
		std::vector<IndMatch> vec_PutativeMatches;

		matching::DistanceRatioMatch(
			0.8, this->matchingType,
			*regions1.get(),
			*regions2.get(),
			vec_PutativeMatches);

		if (!vec_PutativeMatches.empty()) {
			putativeMatches.insert({ pairIdx, std::move(vec_PutativeMatches) });
			overlap.insert({ pairIdx, std::move(vec_PutativeMatches.size()) });
		}
		else
			overlap.insert({ pairIdx, 0 });
	}
}