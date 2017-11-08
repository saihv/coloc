// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2016 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

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

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;

using namespace openMVG::image;
using namespace openMVG::matching_image_collection;
using namespace std;

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

		PairWiseMatches map_PutativesMatches;
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
			map_PutativesMatches.insert({ { 0,1 }, std::move(vec_PutativeMatches) });
			overlap.insert({ { 0, 1 }, std::move(vec_PutativeMatches.size()) });
		}
		else
			overlap.insert({ { 0, 1 }, 0 });
	
		return map_PutativesMatches;
	}
}

int main(int argc, char **argv)
{
	CmdLine cmd;

	std::string sSfM_Data_Filename = "C:\\Users\\svempral\\Desktop\\openMVGtest\\sfm_data.json";
	std::string feat_directory = "C:\\Users\\svempral\\Desktop\\openMVGtest";

	std::string im1 = "img__Quad0_0";
	std::string im2 = "img__Quad1_0";
	
	using namespace openMVG::features;
	std::unique_ptr<Regions> regions_type;
	std::map<IndexT, std::unique_ptr<features::Regions> > regions;

	const std::string sInputDir = "C:\\Users\\svempral\\Desktop\\openMVGtest\\";
	Image<RGBColor> image;
	const string jpg_filenameL = sInputDir + "img__Quad0_0.png";
	const string jpg_filenameR = sInputDir + "img__Quad1_0.png";

	Image<unsigned char> imageL, imageR;
	ReadImage(jpg_filenameL.c_str(), &imageL);
	ReadImage(jpg_filenameR.c_str(), &imageR);

	//--
	// Detect regions thanks to an image_describer
	//--
	using namespace openMVG::features;
	std::unique_ptr<Image_describer> image_describer(new AKAZE_Image_describer_MLDB);
	std::map<IndexT, std::unique_ptr<features::Regions>> regions_perImage;
	image_describer->Describe(imageL, regions[0]);
	image_describer->Describe(imageR, regions[1]);

	const AKAZE_Binary_Regions* regionsL = dynamic_cast<AKAZE_Binary_Regions*>(regions.at(0).get());
	const AKAZE_Binary_Regions* regionsR = dynamic_cast<AKAZE_Binary_Regions*>(regions.at(1).get());


	const PointFeatures
		featsL = regions.at(0)->GetRegionsPositions(),
		featsR = regions.at(1)->GetRegionsPositions();

	Pair_Set pairs = exhaustivePairs(2);
	std::vector<IndMatch> vec_PutativeMatches;
	for (int i = 0; i < pairs.size(); ++i) {
		matching::DistanceRatioMatch(0.8, matching::BRUTE_FORCE_HAMMING, *regions.at(0).get(), *regions.at(1).get(), vec_PutativeMatches);
	}

	Mat xL(2, vec_PutativeMatches.size());
	Mat xR(2, vec_PutativeMatches.size());
	for (size_t k = 0; k < vec_PutativeMatches.size(); ++k) {
		const PointFeature & imaL = featsL[vec_PutativeMatches[k].i_];
		const PointFeature & imaR = featsR[vec_PutativeMatches[k].j_];
		xL.col(k) = imaL.coords().cast<double>();
		xR.col(k) = imaR.coords().cast<double>();
	}

	Mat3 K;
	K << 320, 0, 320, 
		0, 320, 240, 
		0, 0, 1;

	//B. Compute the relative pose thanks to a essential matrix estimation
	std::pair<size_t, size_t> size_imaL(640, 480);
	std::pair<size_t, size_t> size_imaR(640, 480);
	RelativePose_Info relativePose_info;
	if (!robustRelativePose(K, K, xL, xR, relativePose_info, size_imaL, size_imaR, 256))
	{
		std::cerr << " /!\\ Robust relative pose estimation failure."
			<< std::endl;
		return EXIT_FAILURE;
	}
	
	std::cout << "\nFound an Essential matrix:\n"
		<< "\tprecision: " << relativePose_info.found_residual_precision << " pixels\n"
		<< "\t#inliers: " << relativePose_info.vec_inliers.size() << "\n"
		<< "\t#matches: " << vec_PutativeMatches.size()
		<< std::endl;

	return EXIT_SUCCESS;
}
