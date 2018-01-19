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
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/multiview/motion_from_essential.hpp"
#include "openMVG/multiview/solver_essential_kernel.hpp"
#include "openMVG/multiview/solver_essential_eight_point.hpp"
#include "openMVG/numeric/numeric.h"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"
#include "openMVG/sfm/pipelines/localization/SfM_Localizer.hpp"

#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/multiview/solver_resection_kernel.hpp"
#include "openMVG/multiview/solver_resection_p3p.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_BA.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_landmark.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatchDecoratorXY.hpp"
#include "openMVG/matching_image_collection/Geometric_Filter_utils.hpp"
#include "openMVG/multiview/solver_homography_kernel.hpp"
#include "openMVG/robust_estimation/guided_matching.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"
#include "openMVG/sfm/sfm_data.hpp"

#include "localizationParams.hpp"
#include "localizationData.hpp"

using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::matching_image_collection;


#include <iostream>

using namespace openMVG;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::sfm;

namespace coloc
{
	class RobustMatcher {
	public:
		RobustMatcher(LocalizationParams& params)
		{
			this->params = &params;
		}

		std::unique_ptr <RelativePose_Info> computeRelativePose(Pair, FeatureMap& regions, PairWiseMatches& putativeMatches);
		void filterMatches(FeatureMap& regions, PairWiseMatches& putativeMatches, PairWiseMatches& geometricMatches, InterPoseMap& relativePoses);
		bool matchMaps(LocalizationData &data1, LocalizationData &data2, std::vector<IndMatch> &commonFeatures);
		PairWiseMatches geometricMatches;

	private:
		FeatureMap featureRegions;
		std::pair<size_t, size_t> *imageSize;
		int iterationCount = 1024;
		Mat3 *K;
		LocalizationParams *params;

		bool filterHomography(const IntrinsicBase * intrinsics1,
			const IntrinsicBase * intrinsics2,
			const Mat & x1,
			const Mat & x2,
			RelativePose_Info & relativePose_info, LocalizationParams& params);
		bool filterEssential(const IntrinsicBase * intrinsics1,
			const IntrinsicBase * intrinsics2,
			const Mat & x1,
			const Mat & x2,
			RelativePose_Info & relativePose_info, LocalizationParams& params);
	};

	bool RobustMatcher::filterEssential
	(
		const IntrinsicBase * intrinsics1,
		const IntrinsicBase * intrinsics2,
		const Mat & x1,
		const Mat & x2,
		RelativePose_Info & relativePose_info,
		LocalizationParams& params
	)
	{
		if (!intrinsics1 || !intrinsics2)
			return Failure;

		const Mat3X bearing1 = (*intrinsics1)(x1), bearing2 = (*intrinsics2)(x2);

		using KernelType = robust::ACKernelAdaptorEssential<
			openMVG::essential::kernel::FivePointSolver,
			openMVG::fundamental::kernel::EpipolarDistanceError,
			Mat3>;
		KernelType kernel(x1, bearing1, params.imageSize.first, params.imageSize.second,
			x2, bearing2, params.imageSize.first, params.imageSize.second,
			dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics1)->K(),
			dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics2)->K());

		const auto ACRansacOut = ACRANSAC(kernel, relativePose_info.vec_inliers, iterationCount, &relativePose_info.essential_matrix,
			relativePose_info.initial_residual_tolerance, false);

		relativePose_info.found_residual_precision = ACRansacOut.first;

		if (relativePose_info.vec_inliers.size() < 2.5 * KernelType::Solver::MINIMUM_SAMPLES)
			return Failure;

		Pose3 relative_pose;
		if (!RelativePoseFromEssential(bearing1, bearing2, relativePose_info.essential_matrix, relativePose_info.vec_inliers, &relative_pose))
			return Failure;
		
		relativePose_info.relativePose = relative_pose;
		return Success;
	}

	bool RobustMatcher::filterHomography(
		const IntrinsicBase * intrinsics1,
		const IntrinsicBase * intrinsics2,
		const Mat & x1,
		const Mat & x2,
		RelativePose_Info & relativePose_info, LocalizationParams& params)
	{
		using KernelType = robust::ACKernelAdaptor<
			openMVG::homography::kernel::FourPointSolver,
			openMVG::homography::kernel::AsymmetricError,
			UnnormalizerI,
			Mat3>;

		KernelType kernel(
			x1, params.imageSize.first, params.imageSize.second,
			x2, params.imageSize.first, params.imageSize.second,
			false); // configure as point to point error model.

		const double maxPrecision = Square(std::numeric_limits<double>::infinity());
		Mat3 H = Mat3::Identity();
		const std::pair<double, double> ACRansacOut = ACRANSAC(kernel, relativePose_info.vec_inliers, iterationCount, &relativePose_info.essential_matrix, maxPrecision);

		if (relativePose_info.vec_inliers.size() < KernelType::MINIMUM_SAMPLES *2.5)
			return Failure;
		else {
			relativePose_info.found_residual_precision = ACRansacOut.first;

			Vec3 t = relativePose_info.essential_matrix.col(2).normalized();	
			Mat3 R;

			R.col(0) = relativePose_info.essential_matrix.col(0).normalized();
			R.col(1) = relativePose_info.essential_matrix.col(1).normalized();
			R.col(2) = R.col(0).cross(R.col(1));

			relativePose_info.relativePose = Pose3(R, -R.transpose()*t);
			return Success;
		}		
		return Success;
	}

	bool RobustMatcher::matchMaps(LocalizationData &data1, LocalizationData &data2, std::vector<IndMatch> &commonFeatures)
	{
		std::vector <IndMatch> putativeMatches, filteredMatches;

		matching::DistanceRatioMatch(
			0.8, BRUTE_FORCE_HAMMING,
			*data1.mapRegions.get(),
			*data2.mapRegions.get(),
			putativeMatches);

		const PointFeatures featI = data1.mapRegions->GetRegionsPositions();
		const PointFeatures featJ = data2.mapRegions->GetRegionsPositions();

		Mat xL(2, putativeMatches.size());
		Mat xR(2, putativeMatches.size());
		for (size_t k = 0; k < putativeMatches.size(); ++k) {
			xL.col(k) = featI[putativeMatches[k].i_].coords().cast<double>();
			xR.col(k) = featJ[putativeMatches[k].j_].coords().cast<double>();
		}

		RelativePose_Info relativePose;

		const openMVG::cameras::Pinhole_Intrinsic
			camL(params->imageSize.first, params->imageSize.second, (params->K)(0, 0), (params->K)(0, 2), (params->K)(1, 2)),
			camR(params->imageSize.first, params->imageSize.second, (params->K)(0, 0), (params->K)(0, 2), (params->K)(1, 2));

		bool status;

		if (params->filterType == 'H')
			status = filterHomography(&camL, &camR, xL, xR, relativePose, *params);
		else if (params->filterType == 'E')
			status = filterEssential(&camL, &camR, xL, xR, relativePose, *params);
		else {
			std::cout << "Unknown filtering type: aborting." << std::endl;
			return Failure;
		}
		if (!status) 
			std::cerr << "Unable to find any common features between maps." << std::endl;
		else {
			for (int ic = 0; ic < relativePose.vec_inliers.size(); ++ic)
				commonFeatures.push_back(putativeMatches[relativePose.vec_inliers[ic]]);
		}
	}

	std::unique_ptr <RelativePose_Info> RobustMatcher::computeRelativePose(Pair current_pair, FeatureMap& regions, PairWiseMatches& putativeMatches)
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

		const openMVG::cameras::Pinhole_Intrinsic
			camL(params->imageSize.first, params->imageSize.second, (params->K)(0, 0), (params->K)(0, 2), (params->K)(1, 2)),
			camR(params->imageSize.first, params->imageSize.second, (params->K)(0, 0), (params->K)(0, 2), (params->K)(1, 2));

		bool status;

		if (params->filterType == 'H')
			status = filterHomography(&camL, &camR, xL, xR, relativePose, *params);
		else if (params->filterType == 'E')
			status = filterEssential(&camL, &camR, xL, xR, relativePose, *params);
		else {
			std::cout << "Unknown filtering type: aborting." << std::endl;
			return Failure;
		}

		if (!status)
			std::cerr << "Unable to estimate relative pose." << std::endl;

		else {
			std::cout << "\nFound a relative transformation:\n"
				<< "\tprecision: " << relativePose.found_residual_precision << " pixels\n"
				<< "\t#inliers: " << relativePose.vec_inliers.size() << "\n"
				<< "\t#matches: " << pairMatches.size()
				<< std::endl;
		}

		return std::make_unique<RelativePose_Info>(relativePose);
	}

	void RobustMatcher::filterMatches(FeatureMap& regions, PairWiseMatches& putativeMatches, PairWiseMatches& geometricMatches, InterPoseMap& relativePoses)
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

			relativePoses.insert({ currentPair, *relativePose });
		}
	}
}