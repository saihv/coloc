#pragma once

#pragma once
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"
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

#include "third_party/ceres/problem.h"
#include "third_party/ceres/solver.h"
#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/geometry/Similarity3.hpp"
#include "openMVG/geometry/Similarity3_Kernel.hpp"

#include "third_party/ceres/rotation.h"
#include "third_party/ceres/types.h"
#include "third_party/ceres/local_parameterization.h"
#include "third_party/ceres/loss_function.h"
#include "third_party/ceres/covariance.h"

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::sfm;

namespace coloc
{
	class PoseRefiner
	{
	public:
		bool refinementOptions(bool verbose);
		bool refinePose(SfM_Data& scene, const Optimize_Options options, Cov6& poseCovariance);

	private:
		struct CeresOptions
		{
			bool bVerbose_ = true;
			unsigned int nb_threads_ = 1;
			bool bCeres_summary_ = false;
			int linear_solver_type_ = ceres::SPARSE_SCHUR;
			int preconditioner_type_ = ceres::JACOBI;
			int sparse_linear_algebra_library_type_ = ceres::CX_SPARSE;
			double parameter_tolerance_ = 1e-8;
			bool bUse_loss_function_ = true;
		}ceresOptions;
	};

	bool PoseRefiner::refinePose(SfM_Data &scene, const Optimize_Options options, std::vector<std::array<double, 6 * 6> > &poseCovariance = std::vector<std::array<double, 36> >())
	{
		ceres::Problem problem;

		// Data wrapper for refinement:
		Hash_Map<IndexT, std::vector<double>> mapIntrinsics;
		Hash_Map<IndexT, std::vector<double>> mapPoses;

		std::vector<double *> poseParameter;
		ceres::Covariance::Options covOptions;
		ceres::Covariance covariance(covOptions);

		std::vector<std::pair<const double*, const double*> > covariance_blocks;

		// Setup Poses data & subparametrization
		for (const auto & currentPose : scene.poses) {
			const IndexT poseIdx = currentPose.first;
			const Pose3 & pose = currentPose.second;

			const Mat3 R = pose.rotation();
			const Vec3 t = pose.translation();

			double angleAxis[3];
			ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
			// angleAxis + translation
			mapPoses[poseIdx] = { angleAxis[0], angleAxis[1], angleAxis[2], t(0), t(1), t(2) };

			double * parameter_block = &mapPoses[poseIdx][0];
			poseParameter.push_back(parameter_block);
			problem.AddParameterBlock(parameter_block, 6);

			if (scene.poses.size() == 1) {
				covariance_blocks.push_back(std::make_pair(parameter_block, parameter_block));
			}
			else {
				if (options.structure_opt == Structure_Parameter_Type::NONE && currentPose.first == 1)
					covariance_blocks.push_back(std::make_pair(parameter_block, parameter_block));
			}

			if (options.extrinsics_opt == Extrinsic_Parameter_Type::NONE)
				problem.SetParameterBlockConstant(parameter_block);
			
			else {
				std::vector<int> vec_constant_extrinsic;
				// If we adjust only the translation, we must set ROTATION as constant
				if (options.extrinsics_opt == Extrinsic_Parameter_Type::ADJUST_TRANSLATION)
					vec_constant_extrinsic.insert(vec_constant_extrinsic.end(), { 0,1,2 });
				
				if (options.extrinsics_opt == Extrinsic_Parameter_Type::ADJUST_ROTATION)
					vec_constant_extrinsic.insert(vec_constant_extrinsic.end(), { 3,4,5 });
				
				if (!vec_constant_extrinsic.empty()) {
					ceres::SubsetParameterization *subset_parameterization =
						new ceres::SubsetParameterization(6, vec_constant_extrinsic);
					problem.SetParameterization(parameter_block, subset_parameterization);
				}
			}
		}

		// Setup Intrinsics data & subparametrization
		for (const auto & currentIntrinsic : scene.intrinsics) {
			const IndexT indexCam = currentIntrinsic.first;

			if (isValid(currentIntrinsic.second->getType())) {
				mapIntrinsics[indexCam] = currentIntrinsic.second->getParams();
				if (!mapIntrinsics.at(indexCam).empty()) {
					double * parameter_block = &mapIntrinsics.at(indexCam)[0];
					problem.AddParameterBlock(parameter_block, mapIntrinsics.at(indexCam).size());
					if (options.intrinsics_opt == Intrinsic_Parameter_Type::NONE) 
						problem.SetParameterBlockConstant(parameter_block);
				}
			}
		}

		ceres::LossFunction * lossFunction = ceresOptions.bUse_loss_function_ ? new ceres::HuberLoss(Square(4.0)) : nullptr;

		for (auto & landmark : scene.structure) {
			const Observations & observation = landmark.second.obs;

			for (const auto & obs_it : observation) {
				// Build the residual block corresponding to the track observation:
				const View * view = scene.views.at(obs_it.first).get();

				// Each Residual block takes a point and a camera as input and outputs a 2
				// dimensional residual. Internally, the cost function stores the observed
				// image location and compares the reprojection against the observation.
				ceres::CostFunction* cost_function = IntrinsicsToCostFunction(scene.intrinsics.at(view->id_intrinsic).get(),
					obs_it.second.x);

				if (cost_function) {
					if (!mapIntrinsics.at(view->id_intrinsic).empty())
						problem.AddResidualBlock(cost_function, lossFunction, &mapIntrinsics.at(view->id_intrinsic)[0],
							&mapPoses.at(view->id_pose)[0], landmark.second.X.data());
					
					else 
						problem.AddResidualBlock(cost_function, lossFunction, &mapPoses.at(view->id_pose)[0],
							landmark.second.X.data());
				}
				else {
					std::cerr << "Cannot create a CostFunction for this camera model." << std::endl;
					return false;
				}
			}
			if (options.structure_opt == Structure_Parameter_Type::NONE)
				problem.SetParameterBlockConstant(landmark.second.X.data());
		}


		// Configure a BA engine and run it
		//  Make Ceres automatically detect the bundle structure.
		ceres::Solver::Options ceresConfig;
		ceresConfig.max_num_iterations = 500;
		ceresConfig.preconditioner_type = static_cast<ceres::PreconditionerType>(ceresOptions.preconditioner_type_);
		ceresConfig.linear_solver_type = static_cast<ceres::LinearSolverType>(ceresOptions.linear_solver_type_);
		ceresConfig.sparse_linear_algebra_library_type = static_cast<ceres::SparseLinearAlgebraLibraryType>(ceresOptions.sparse_linear_algebra_library_type_);
		ceresConfig.minimizer_progress_to_stdout = ceresOptions.bVerbose_;
		ceresConfig.logging_type = ceres::SILENT;
		ceresConfig.num_threads = ceresOptions.nb_threads_;
		ceresConfig.num_linear_solver_threads = ceresOptions.nb_threads_;
		ceresConfig.parameter_tolerance = ceresOptions.parameter_tolerance_;

		// Solve BA
		ceres::Solver::Summary summary;
		ceres::Solve(ceresConfig, &problem, &summary);
		if (ceresOptions.bCeres_summary_)
			std::cout << summary.FullReport() << std::endl;

		if (covariance_blocks.size() != 0) {
			covariance.Compute(covariance_blocks, &problem);
			if (poseParameter.size() > 1) {
				std::array<double, 6 * 6> covpose;
				double cov_pose[6 * 6];
				covariance.GetCovarianceBlock(poseParameter[1], poseParameter[1], cov_pose);
				std::copy(std::begin(cov_pose), std::end(cov_pose), std::begin(covpose));
				poseCovariance.push_back(covpose);
			}
			else {
				std::array<double, 6 * 6> covpose;
				double cov_pose[6 * 6];
				covariance.GetCovarianceBlock(poseParameter[0], poseParameter[0], cov_pose);
				std::copy(std::begin(cov_pose), std::end(cov_pose), std::begin(covpose));
				poseCovariance.push_back(covpose);
			}
		}		

		if (!summary.IsSolutionUsable()) {
			if (ceresOptions.bVerbose_)
				std::cout << "Bundle Adjustment failed." << std::endl;
			return false;
		}
		else {
			if (ceresOptions.bVerbose_) {
				std::cout << std::endl
					<< "Pose/scene refinement statistics:\n"
					<< " #views: " << scene.views.size() << "\n"
					<< " #poses: " << scene.poses.size() << "\n"
					<< " #landmarks: " << scene.structure.size() << "\n"
					<< " #residuals: " << summary.num_residuals << "\n"
					<< " Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
					<< " Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
					<< " Time (s): " << summary.total_time_in_seconds << "\n"
					<< std::endl;
			}

			// Update camera poses with refined data
			if (options.extrinsics_opt != Extrinsic_Parameter_Type::NONE) {
				for (auto & currentPose : scene.poses) {
					const IndexT poseIdx = currentPose.first;
					Mat3 R_refined;
					ceres::AngleAxisToRotationMatrix(&mapPoses.at(poseIdx)[0], R_refined.data());
					Vec3 t_refined(mapPoses.at(poseIdx)[3], mapPoses.at(poseIdx)[4], mapPoses.at(poseIdx)[5]);
					// Update the pose
					Pose3 & pose = currentPose.second;
					pose = Pose3(R_refined, -R_refined.transpose() * t_refined);
				}
			}
			return true;
		}
	}
}