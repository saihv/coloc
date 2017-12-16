#pragma once

#include "openMVG/image/image_io.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"

#include <iostream>

using namespace openMVG;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::sfm;

namespace coloc
{
	class RobustMatcher {
	public:
		RobustMatcher(int, int);

		std::unique_ptr <RelativePose_Info> computeRelativePose(Pair, std::map<IndexT, std::unique_ptr<features::Regions> >&, PairWiseMatches&);
		void filterMatches(std::map<IndexT, std::unique_ptr<features::Regions> >& regions, PairWiseMatches& putativeMatches, PairWiseMatches& geometricMatches);

		PairWiseMatches geometricMatches;

	private:
		std::map<IndexT, std::unique_ptr<features::Regions> > featureRegions;
		std::pair<size_t, size_t> imageSize;

		int iterationCount = 256;

		Mat3 K;
	};

	RobustMatcher::RobustMatcher(int w, int h)
	{
		imageSize.first = w;
		imageSize.second = h;


		K << 320, 0, 320,
			0, 320, 240,
			0, 0, 1;
	}

	std::unique_ptr <RelativePose_Info> RobustMatcher::computeRelativePose(Pair current_pair, std::map<IndexT, std::unique_ptr<features::Regions> >& regions, PairWiseMatches& putativeMatches)
	{
		const uint32_t I = std::min(current_pair.first, current_pair.second);
		const uint32_t J = std::max(current_pair.first, current_pair.second);

		const PointFeatures featI = regions.at(I)->GetRegionsPositions();
		const PointFeatures featJ = regions.at(J)->GetRegionsPositions();

		std::vector <IndMatch> pairMatches = putativeMatches[current_pair];
		Mat xL(2, pairMatches.size());
		Mat xR(2, pairMatches.size());
		for (size_t k = 0; k < pairMatches.size(); ++k) {
			xL.col(k) = featI[pairMatches[k].i_].coords().cast<double>();
			xR.col(k) = featJ[pairMatches[k].j_].coords().cast<double>();
		}

		RelativePose_Info relativePose;

		if (!robustRelativePose(K, K, xL, xR, relativePose, imageSize, imageSize, iterationCount))
		{
			std::cerr << " /!\\ Robust relative pose estimation failure."
				<< std::endl;
		}

		else
		{
			std::cout << "\nFound an Essential matrix:\n"
				<< "\tprecision: " << relativePose.found_residual_precision << " pixels\n"
				<< "\t#inliers: " << relativePose.vec_inliers.size() << "\n"
				<< "\t#matches: " << pairMatches.size()
				<< std::endl;
		}

		return std::make_unique<RelativePose_Info>(relativePose);
	}

	void RobustMatcher::filterMatches(std::map<IndexT, std::unique_ptr<features::Regions> >& regions, PairWiseMatches& putativeMatches, PairWiseMatches& geometricMatches)
	{
		for (const auto matchedPair : putativeMatches) {
			Pair currentPair = matchedPair.first;
			std::vector <IndMatch> pairMatches = putativeMatches[currentPair];
			std::unique_ptr<RelativePose_Info> relativePose = computeRelativePose(matchedPair.first, regions, putativeMatches);

			std::vector <IndMatch> vec_geometricMatches;
			for (int ic = 0; ic < relativePose->vec_inliers.size(); ++ic) 
				vec_geometricMatches.push_back(pairMatches[relativePose->vec_inliers[ic]]);

			if (!vec_geometricMatches.empty())
				geometricMatches.insert({ { currentPair.first, currentPair.second }, std::move(vec_geometricMatches) });

			//relativePoses.insert({ { pair_idx.first, pair_idx.second }, relativePose_info });
		}
	}
}