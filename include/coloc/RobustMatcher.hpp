//
// Created by Sai Vemprala on 7/8/18.
//

#include "colocParams.hpp"
#include "colocData.hpp"

#include "openMVG/multiview/motion_from_essential.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"

#include <iostream>

using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::matching_image_collection;
using namespace openMVG;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::sfm;

namespace coloc
{
	class RobustMatcher {
	public:
		RobustMatcher(colocParams& params)
		{
			this->params = &params;
		}

	private:
		std::pair<int, int> *imageSize;
		int iterationCount = 256;
		std::vector <Mat3> *K;
		colocParams *params;

	public:
		bool performChiralityTest(const Mat3X & x1,
			const Mat3X & x2,
			const Mat3 & E,
			const std::vector<uint32_t> & bearing_vector_index_to_use,
			std::vector <Pose3> relative_poses,
			Pose3 * final_pose,
			std::vector<uint32_t> * vec_selected_points = nullptr,
			std::vector<Vec3> * vec_points = nullptr,
			const double positive_depth_solution_ratio = 0.7)
		{
			std::vector<uint32_t> cheirality_accumulator(relative_poses.size(), 0);

			// Find which solution is the best:
			// - count how many triangulated observations are in front of the cameras
			std::vector<std::vector<uint32_t>> vec_newInliers(relative_poses.size());
			std::vector<std::vector<Vec3>> vec_3D(relative_poses.size());

			const Pose3 pose1(Mat3::Identity(), Vec3::Zero());
			const Mat34 P1 = pose1.asMatrix();

			for (size_t i = 0; i < relative_poses.size(); ++i)
			{
				const Pose3 pose2 = relative_poses[i];
				const Mat34 P2 = pose2.asMatrix();
				Vec3 X;

				for (const uint32_t inlier_idx : bearing_vector_index_to_use)
				{
					const auto
						f1 = x1.col(inlier_idx),
						f2 = x2.col(inlier_idx);
					TriangulateDLT(P1, f1, P2, f2, &X);
					// Test if X is visible by the two cameras
					if (CheiralityTest(f1, pose1, f2, pose2, X))
					{
						++cheirality_accumulator[i];
						vec_newInliers[i].push_back(inlier_idx);
						vec_3D[i].push_back(X);
					}
				}
			}

			// Check if there is a valid solution:
			const auto iter = std::max_element(cheirality_accumulator.cbegin(), cheirality_accumulator.cend());
			if (*iter == 0)
			{
				// There is no right solution with points in front of the cameras
				return false;
			}

			// Export the best solution data
			const size_t index = std::distance(cheirality_accumulator.cbegin(), iter);
			if (final_pose)
			{
				(*final_pose) = relative_poses[index];
			}
			if (vec_selected_points)
				(*vec_selected_points) = vec_newInliers[index];
			if (vec_points)
				(*vec_points) = vec_3D[index];

			// Test if the best solution is good by using the ratio of the two best solution score
			std::sort(cheirality_accumulator.begin(), cheirality_accumulator.end());
			const double ratio = cheirality_accumulator.rbegin()[1] / static_cast<double>(cheirality_accumulator.rbegin()[0]);
			return (ratio < positive_depth_solution_ratio);
		}

		bool decomposeHomography(Mat3& H, std::vector <Pose3> & motions)
		{
			cv::Mat Homography(3, 3, CV_64FC1), intrinsics(3, 3, CV_64FC1);
			std::vector<cv::Mat> r, t, n, m;
			cv::eigen2cv(H, Homography);
			cv::eigen2cv(params->K[0], intrinsics);
			cv::decomposeHomographyMat(Homography, intrinsics, r, t, n);

			for (size_t i = 0; i < r.size(); ++i) {
				Mat3 Rot;
				Vec3 trn;

				cv::cv2eigen(r[i], Rot);
				trn[0] = t[i].at<double>(0);
				trn[1] = t[i].at<double>(1);
				trn[2] = t[i].at<double>(2);

				motions.emplace_back(Rot, trn.normalized());
			}
			return EXIT_SUCCESS;
		}

		bool filterFundamental(const IntrinsicBase * intrinsics1, const IntrinsicBase * intrinsics2, const Mat & x1, const Mat & x2,
			RelativePose_Info & relativePose_info, colocParams& params, bool findPose)
		{
			if (!intrinsics1 || !intrinsics2)
				return EXIT_FAILURE;

			using KernelType = robust::ACKernelAdaptor<
				openMVG::fundamental::kernel::SevenPointSolver,
				openMVG::fundamental::kernel::EpipolarDistanceError,
				UnnormalizerT, Mat3>;

			KernelType kernel(x1, params.imageSize.first, params.imageSize.second,
				x2, params.imageSize.first, params.imageSize.second, true);

			const auto ACRansacOut = ACRANSAC(kernel, relativePose_info.vec_inliers, iterationCount, &relativePose_info.essential_matrix,
				relativePose_info.initial_residual_tolerance, false);

			relativePose_info.found_residual_precision = 5.0;

			if (relativePose_info.vec_inliers.size() < 2.5 * KernelType::Solver::MINIMUM_SAMPLES)
				return EXIT_FAILURE;

			return EXIT_SUCCESS;
		}

		bool filterEssential(const IntrinsicBase * intrinsics1, const IntrinsicBase * intrinsics2, const Mat & x1, const Mat & x2,
			RelativePose_Info & relativePose_info, colocParams& params, bool findPose)
		{
			if (!intrinsics1 || !intrinsics2)
				return EXIT_FAILURE;

			const Mat3X norm2Dpt_1 = (*intrinsics1)(x1), norm2Dpt_2 = (*intrinsics2)(x2);

			using KernelType = robust::ACKernelAdaptorEssential<
				openMVG::essential::kernel::FivePointSolver,
				openMVG::fundamental::kernel::EpipolarDistanceError,
				Mat3>;
			KernelType kernel(x1, norm2Dpt_1, params.imageSize.first, params.imageSize.second,
				x2, norm2Dpt_2, params.imageSize.first, params.imageSize.second,
				dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics1)->K(),
				dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics2)->K());

			const auto ACRansacOut = ACRANSAC(kernel, relativePose_info.vec_inliers, iterationCount, &relativePose_info.essential_matrix,
				relativePose_info.initial_residual_tolerance, false);

			relativePose_info.found_residual_precision = 5.0;

			if (relativePose_info.vec_inliers.size() < 2.5 * KernelType::Solver::MINIMUM_SAMPLES)
				return EXIT_FAILURE;

			if (findPose) {
				Pose3 relative_pose;
				if (!RelativePoseFromEssential(norm2Dpt_1, norm2Dpt_2, relativePose_info.essential_matrix, relativePose_info.vec_inliers, &relative_pose))
					return EXIT_FAILURE;

				relativePose_info.relativePose = relative_pose;
			}
			return EXIT_SUCCESS;
		}

		bool filterHomography(const IntrinsicBase * intrinsics1, const IntrinsicBase * intrinsics2, const Mat & x1, const Mat & x2,
			RelativePose_Info & relativePose_info, colocParams& params, bool findPose)
		{
			using KernelType = robust::ACKernelAdaptor<
				openMVG::homography::kernel::FourPointSolver,
				openMVG::homography::kernel::AsymmetricError,
				UnnormalizerI,
				Mat3>;

			const Mat3X normpt2D_1 = (*intrinsics1)(x1), normpt2D_2 = (*intrinsics2)(x2);

			KernelType kernel(
				x1, params.imageSize.first, params.imageSize.second,
				x2, params.imageSize.first, params.imageSize.second,
				false); // configure as point to point error model.

			const double maxPrecision = Square(std::numeric_limits<double>::infinity());
			Mat3 H = Mat3::Identity();
			const std::pair<double, double> ACRansacOut = ACRANSAC(kernel, relativePose_info.vec_inliers, iterationCount, &relativePose_info.essential_matrix, maxPrecision);

			// relativePose_info.found_residual_precision = 5.0;

			if (relativePose_info.vec_inliers.size() < KernelType::MINIMUM_SAMPLES * 2.5)
				return EXIT_FAILURE;
			else {
				relativePose_info.found_residual_precision = ACRansacOut.first;

				if (findPose) {
					std::vector <Pose3> motions;
					decomposeHomography(relativePose_info.essential_matrix, motions);
					Pose3 final_pose;

					performChiralityTest(normpt2D_1, normpt2D_2, relativePose_info.essential_matrix, relativePose_info.vec_inliers, motions, &final_pose);
					relativePose_info.relativePose = final_pose;

					// std::cout << final_pose.center() << std::endl;

					relativePose_info.found_residual_precision = ACRansacOut.first;
					/*
					Vec3 t = relativePose_info.essential_matrix.col(2).normalized();
					Mat3 R;

					R.col(0) = relativePose_info.essential_matrix.col(0).normalized();
					R.col(1) = relativePose_info.essential_matrix.col(1).normalized();
					R.col(2) = R.col(0).cross(R.col(1));

					relativePose_info.relativePose = Pose3(R, -R.transpose()*t);
					*/
				}
			}
			return EXIT_SUCCESS;
		}

		bool matchMaps(std::unique_ptr<features::AKAZE_Binary_Regions> &scene1, std::unique_ptr<features::AKAZE_Binary_Regions> &scene2, std::vector<IndMatch> &commonFeatures, Vec3& poseDiff, Mat3& rotDiff)
		{
			std::vector <IndMatch> putativeMatches, filteredMatches;

			putativeMatches = commonFeatures;

			commonFeatures.clear();

			//matching::DistanceRatioMatch(
			//	0.8, BRUTE_FORCE_HAMMING,
			//	*scene1.get(),
			//	*scene2.get(),
			//	putativeMatches);
			
			const PointFeatures featI = scene1->GetRegionsPositions();
			const PointFeatures featJ = scene2->GetRegionsPositions();

			Mat xL(2, putativeMatches.size());
			Mat xR(2, putativeMatches.size());
			for (size_t k = 0; k < putativeMatches.size(); ++k) {
				xL.col(k) = featI[putativeMatches[k].i_].coords().cast<double>();
				xR.col(k) = featJ[putativeMatches[k].j_].coords().cast<double>();
			}

			cv::Mat xLcv, xRcv, Fcv;
			cv::eigen2cv(xL, xLcv);
			cv::eigen2cv(xR, xRcv);


			RelativePose_Info relativePose;

			const openMVG::cameras::Pinhole_Intrinsic_Radial_K3
				camL(params->imageSize.first, params->imageSize.second, (params->K[0])(0, 0), (params->K[0])(0, 2), (params->K[0])(1, 2),
				(params->dist[0])(0), (params->dist[0])(1), (params->dist[0])(2)),
				camR(params->imageSize.first, params->imageSize.second, (params->K[0])(0, 0), (params->K[0])(0, 2), (params->K[0])(1, 2),
				(params->dist[0])(0), (params->dist[0])(1), (params->dist[0])(2));

			Mat A;

			A = camL.K() * rotDiff * poseDiff;
			Eigen::Matrix3d C;
			C << 0, -A(2), A(1),
				A(2), 0, -A(0),
				-A(1), A(0), 0;

			Eigen::Matrix3d F;
			F = (camR.Kinv()).transpose() * rotDiff * (camL.K()).transpose() * C;
			cv::eigen2cv(F, Fcv);

			IndMatches inliers;
			std::ofstream myfile;
			myfile.open("guidedmatches2.txt");
			for (size_t k = 0; k < putativeMatches.size(); ++k) {
				Vec3 f1, f2;
				f1 << xL(0, k), xL(1, k), 1;
				f2 << xR(0, k), xR(1, k), 1;
				Mat res = f1.transpose() * F * f2;

				//if (std::abs(res(0, 0)) < 50.0) {
					std::cout << "Value: " << std::abs(res(0, 0)) << std::endl;
					std::cout << "Left coordinates: " << xL.col(k) << std::endl;
					std::cout << "Right coordinates: " << xR.col(k) << std::endl;
					inliers.push_back(putativeMatches[k]);
				//}

					myfile << res(0, 0) << "," << xL(0, k) << "," << xL(1, k) << "," << xR(0, k) << "," << xR(1, k) << std::endl;

				std::cout << std::abs(res(0,0)) << std::endl;
			}
			myfile.close();

			for (int ic = 0; ic < inliers.size(); ++ic)
				commonFeatures.push_back(inliers[ic]);
			
			return EXIT_SUCCESS;
		}

		std::unique_ptr <RelativePose_Info> computeRelativePose(Pair current_pair, FeatureMap& regions, PairWiseMatches& putativeMatches)
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

			const openMVG::cameras::Pinhole_Intrinsic_Radial_K3
				camL(params->imageSize.first, params->imageSize.second, (params->K[current_pair.first])(0, 0), (params->K[current_pair.first])(0, 2), (params->K[current_pair.first])(1, 2), 
					(params->dist[current_pair.first])(0), (params->dist[current_pair.first])(1), (params->dist[current_pair.first])(2)),
				camR(params->imageSize.first, params->imageSize.second, (params->K[current_pair.second])(0, 0), (params->K[current_pair.second])(0, 2), (params->K[current_pair.second])(1, 2),
					(params->dist[current_pair.second])(0), (params->dist[current_pair.second])(1), (params->dist[current_pair.second])(2));

			bool status, findPose = true;

			if (params->model == 'H')
				status = filterHomography(&camL, &camR, xL, xR, relativePose, *params, findPose);
			else if (params->model == 'E')
				status = filterEssential(&camL, &camR, xL, xR, relativePose, *params, findPose);
			else if (params->model == 'F')
				status = filterFundamental(&camL, &camR, xL, xR, relativePose, *params, findPose);
			else {
				std::cout << "Unknown filtering type: aborting." << std::endl;
			}

			if (status == EXIT_FAILURE)
				std::cerr << "Unable to estimate relative pose." << std::endl;

			else {
				std::cout << "\nFound a relative transformation:\n"
					<< "\tprecision: " << relativePose.found_residual_precision << " pixels\n"
					<< "\t#inliers: " << relativePose.vec_inliers.size() << "\n"
					<< "\t#matches: " << pairMatches.size() << "\n"
					<< "Rotation: " << relativePose.relativePose.rotation() << "\n"
					<< "Translation: " << relativePose.relativePose.center() 
					<< std::endl;
			}

			return std::make_unique<RelativePose_Info>(relativePose);
		}

		void filterMatches(FeatureMap& regions, PairWiseMatches& putativeMatches, PairWiseMatches& geometricMatches, InterPoseMap& relativePoses)
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
	};
}