#pragma once

#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/image_describer_akaze_io.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "openMVG/matching/regions_matcher.hpp"

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::sfm;

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

		PairWiseMatches putativeMatches;
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
			putativeMatches.insert({ { 0,1 }, std::move(vec_PutativeMatches) });
			overlap.insert({ { 0, 1 }, std::move(vec_PutativeMatches.size()) });
		}
		else
			overlap.insert({ { 0, 1 }, 0 });

		return putativeMatches;
	}
}
