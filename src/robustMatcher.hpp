#pragma once

#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"

using namespace openMVG;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::sfm;

namespace coloc
{
	class RobustMatcher {
	public:
		RobustMatcher(std::map<IndexT, std::unique_ptr<features::Regions> >& regions);

		RelativePose_Info computeRelativePose(Pair, PairWiseMatches&);
		std::vector <IndMatch> filterMatches(Pair, RelativePose_Info&);

		PairWiseMatches geometricMatches;

	private:
		std::map<IndexT, std::unique_ptr<features::Regions> > featureRegions;
	};

	RobustMatcher::RobustMatcher(std::map<IndexT, std::unique_ptr<features::Regions> >& regions)
	{
		featureRegions = regions;
	}

	RelativePose_Info RobustMatcher::computeRelativePose(Pair current_pair, PairWiseMatches& putativeMatches) 
	{
		const uint32_t I = std::min(current_pair.first, current_pair.second);
		const uint32_t J = std::max(current_pair.first, current_pair.second);

		const PointFeatures featI = featureRegions.at(I)->GetRegionsPositions();
		const PointFeatures featJ = featureRegions.at(J)->GetRegionsPositions();

		std::vector <IndMatch> pairMatches = putativeMatches[current_pair];
		Mat xL(2, putativeMatches.size());
		Mat xR(2, putativeMatches.size());
		for (size_t k = 0; k < putativeMatches.size(); ++k) {
			xL.col(k) = featI[pairMatches[k].i_].coords().cast<double>();
			xR.col(k) = featJ[pairMatches[k].j_].coords().cast<double>();
		}
	}

	std::vector <IndMatch> RobustMatcher::filterMatches(Pair pair, RelativePose_Info& pose)
	{
		std::vector <IndMatch> vec_geometricMatches;
		for (int ic = 0; ic < pose.vec_inliers.size(); ++ic) {
			vec_geometricMatches.push_back(putativeMatches[relativePose_info.vec_inliers[ic]]);
		}
	}
}