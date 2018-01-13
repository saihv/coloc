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
		bool refinePose(SfM_Data& scene, const Optimize_Options options);

	private:
		struct CeresOptions
		{
			bool bVerbose_ = 1;
			unsigned int nb_threads_ = 4;
			bool bCeres_summary_ = 1;
			int linear_solver_type_ = ceres::SPARSE_SCHUR;
			int preconditioner_type_ = ceres::JACOBI;
			int sparse_linear_algebra_library_type_ = ceres::CX_SPARSE;
			double parameter_tolerance_ = 1e-8;
			bool bUse_loss_function_ = 1;
		};

		CeresOptions ceresOptions;
	};

	bool PoseRefiner::refinePose(SfM_Data& scene, const Optimize_Options options)
	{
		double pose_center_robust_fitting_error = 0.0;
		openMVG::geometry::Similarity3 sim_to_center;
		bool b_usable_prior = false;

		ceres::Problem problem;

		// Data wrapper for refinement:
		Hash_Map<IndexT, std::vector<double>> map_intrinsics;
		Hash_Map<IndexT, std::vector<double>> map_poses;

		std::vector<double *> pose_parameter;
		ceres::Covariance::Options covOptions;
		ceres::Covariance covariance(covOptions);

		std::vector<std::pair<const double*, const double*> > covariance_blocks;

		// Setup Poses data & subparametrization
		for (const auto & pose_it : scene.poses)
		{
			const IndexT indexPose = pose_it.first;

			const Pose3 & pose = pose_it.second;
			const Mat3 R = pose.rotation();
			const Vec3 t = pose.translation();

			double angleAxis[3];
			ceres::RotationMatrixToAngleAxis((const double*)R.data(), angleAxis);
			// angleAxis + translation
			map_poses[indexPose] = { angleAxis[0], angleAxis[1], angleAxis[2], t(0), t(1), t(2) };

			double * parameter_block = &map_poses[indexPose][0];
			pose_parameter.push_back(parameter_block);
			problem.AddParameterBlock(parameter_block, 6);
			covariance_blocks.push_back(std::make_pair(parameter_block, parameter_block));

			if (options.extrinsics_opt == Extrinsic_Parameter_Type::NONE)
			{
				// set the whole parameter block as constant for best performance
				problem.SetParameterBlockConstant(parameter_block);
			}
			else  // Subset parametrization
			{
				std::vector<int> vec_constant_extrinsic;
				// If we adjust only the translation, we must set ROTATION as constant
				if (options.extrinsics_opt == Extrinsic_Parameter_Type::ADJUST_TRANSLATION)
				{
					// Subset rotation parametrization
					vec_constant_extrinsic.insert(vec_constant_extrinsic.end(), { 0,1,2 });
				}
				// If we adjust only the rotation, we must set TRANSLATION as constant
				if (options.extrinsics_opt == Extrinsic_Parameter_Type::ADJUST_ROTATION)
				{
					// Subset translation parametrization
					vec_constant_extrinsic.insert(vec_constant_extrinsic.end(), { 3,4,5 });
				}
				if (!vec_constant_extrinsic.empty())
				{
					ceres::SubsetParameterization *subset_parameterization =
						new ceres::SubsetParameterization(6, vec_constant_extrinsic);
					problem.SetParameterization(parameter_block, subset_parameterization);
				}
			}
		}

		// Setup Intrinsics data & subparametrization
		for (const auto & intrinsic_it : scene.intrinsics)
		{
			const IndexT indexCam = intrinsic_it.first;

			if (isValid(intrinsic_it.second->getType()))
			{
				map_intrinsics[indexCam] = intrinsic_it.second->getParams();
				if (!map_intrinsics.at(indexCam).empty())
				{
					double * parameter_block = &map_intrinsics.at(indexCam)[0];
					problem.AddParameterBlock(parameter_block, map_intrinsics.at(indexCam).size());
					if (options.intrinsics_opt == Intrinsic_Parameter_Type::NONE)
					{
						// set the whole parameter block as constant for best performance
						problem.SetParameterBlockConstant(parameter_block);
					}
				}
			}
		}

		// Set a LossFunction to be less penalized by false measurements
		//  - set it to nullptr if you don't want use a lossFunction.
		ceres::LossFunction * p_LossFunction = ceresOptions.bUse_loss_function_ ? new ceres::HuberLoss(Square(4.0)) : nullptr;

		// For all visibility add reprojections errors:
		for (auto & structure_landmark_it : scene.structure) {
			const Observations & obs = structure_landmark_it.second.obs;

			for (const auto & obs_it : obs) {
				// Build the residual block corresponding to the track observation:
				const View * view = scene.views.at(obs_it.first).get();

				// Each Residual block takes a point and a camera as input and outputs a 2
				// dimensional residual. Internally, the cost function stores the observed
				// image location and compares the reprojection against the observation.
				ceres::CostFunction* cost_function = IntrinsicsToCostFunction(scene.intrinsics.at(view->id_intrinsic).get(),
					obs_it.second.x);

				if (cost_function) {
					if (!map_intrinsics.at(view->id_intrinsic).empty()) {
						problem.AddResidualBlock(cost_function, p_LossFunction, &map_intrinsics.at(view->id_intrinsic)[0],
							&map_poses.at(view->id_pose)[0], structure_landmark_it.second.X.data());
					}
					else {
						problem.AddResidualBlock(cost_function, p_LossFunction, &map_poses.at(view->id_pose)[0],
							structure_landmark_it.second.X.data());
					}
				}
				else {
					std::cerr << "Cannot create a CostFunction for this camera model." << std::endl;
					return false;
				}
			}
			if (options.structure_opt == Structure_Parameter_Type::NONE)
				problem.SetParameterBlockConstant(structure_landmark_it.second.X.data());
		}


		// Configure a BA engine and run it
		//  Make Ceres automatically detect the bundle structure.
		ceres::Solver::Options ceres_config_options;
		ceres_config_options.max_num_iterations = 500;
		ceres_config_options.preconditioner_type = static_cast<ceres::PreconditionerType>(ceresOptions.preconditioner_type_);
		ceres_config_options.linear_solver_type = static_cast<ceres::LinearSolverType>(ceresOptions.linear_solver_type_);
		ceres_config_options.sparse_linear_algebra_library_type = static_cast<ceres::SparseLinearAlgebraLibraryType>(ceresOptions.sparse_linear_algebra_library_type_);
		ceres_config_options.minimizer_progress_to_stdout = ceresOptions.bVerbose_;
		ceres_config_options.logging_type = ceres::SILENT;
		ceres_config_options.num_threads = ceresOptions.nb_threads_;
		ceres_config_options.num_linear_solver_threads = ceresOptions.nb_threads_;
		ceres_config_options.parameter_tolerance = ceresOptions.parameter_tolerance_;

		// Solve BA
		ceres::Solver::Summary summary;
		ceres::Solve(ceres_config_options, &problem, &summary);
		if (ceresOptions.bCeres_summary_)
			std::cout << summary.FullReport() << std::endl;

		covariance.Compute(covariance_blocks, &problem);

		std::vector<std::array<double, 6 * 6>> covariance_pose;

		for (auto pose_param : pose_parameter)
		{
			std::array<double, 6 * 6> covpose;
			double cov_pose[36];
			covariance.GetCovarianceBlock(pose_param, pose_param, cov_pose);
			std::copy(std::begin(cov_pose), std::end(cov_pose), std::begin(covpose));
			covariance_pose.push_back(covpose);
		}

		// If no error, get back refined parameters
		if (!summary.IsSolutionUsable())
		{
			if (ceresOptions.bVerbose_)
				std::cout << "Bundle Adjustment failed." << std::endl;
			return false;
		}
		else // Solution is usable
		{
			if (ceresOptions.bVerbose_) {
				std::cout << std::endl
					<< "Bundle Adjustment statistics (approximated RMSE):\n"
					<< " #views: " << scene.views.size() << "\n"
					<< " #poses: " << scene.poses.size() << "\n"
					<< " #intrinsics: " << scene.intrinsics.size() << "\n"
					<< " #tracks: " << scene.structure.size() << "\n"
					<< " #residuals: " << summary.num_residuals << "\n"
					<< " Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
					<< " Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
					<< " Time (s): " << summary.total_time_in_seconds << "\n"
					<< std::endl;
				if (options.use_motion_priors_opt)
					std::cout << "Usable motion priors: " << (int)b_usable_prior << std::endl;
			}

			// Update camera poses with refined data
			if (options.extrinsics_opt != Extrinsic_Parameter_Type::NONE) {
				for (auto & pose_it : scene.poses)
				{
					const IndexT indexPose = pose_it.first;

					Mat3 R_refined;
					ceres::AngleAxisToRotationMatrix(&map_poses.at(indexPose)[0], R_refined.data());
					Vec3 t_refined(map_poses.at(indexPose)[3], map_poses.at(indexPose)[4], map_poses.at(indexPose)[5]);
					// Update the pose
					Pose3 & pose = pose_it.second;
					pose = Pose3(R_refined, -R_refined.transpose() * t_refined);
				}
			}
			return true;
		}
	}
}