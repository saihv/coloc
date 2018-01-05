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
			bool bVerbose_;
			unsigned int nb_threads_;
			bool bCeres_summary_;
			int linear_solver_type_;
			int preconditioner_type_;
			int sparse_linear_algebra_library_type_;
			double parameter_tolerance_;
			bool bUse_loss_function_;
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

			double * parameter_block = &map_poses.at(indexPose)[0];
			problem.AddParameterBlock(parameter_block, 6);
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

		// Set a LossFunction to be less penalized by false measurements
		//  - set it to nullptr if you don't want use a lossFunction.
		ceres::LossFunction * p_LossFunction =
			ceresOptions.bUse_loss_function_ ?
			new ceres::HuberLoss(Square(4.0))
			: nullptr;

		// For all visibility add reprojections errors:
		for (auto & structure_landmark_it : scene.structure)
		{
			const Observations & obs = structure_landmark_it.second.obs;

			for (const auto & obs_it : obs)
			{
				// Build the residual block corresponding to the track observation:
				const View * view = scene.views.at(obs_it.first).get();

				// Each Residual block takes a point and a camera as input and outputs a 2
				// dimensional residual. Internally, the cost function stores the observed
				// image location and compares the reprojection against the observation.
				ceres::CostFunction* cost_function =
					IntrinsicsToCostFunction(scene.intrinsics.at(view->id_intrinsic).get(),
						obs_it.second.x);

				if (cost_function)
				{
					if (!map_intrinsics.at(view->id_intrinsic).empty())
					{
						problem.AddResidualBlock(cost_function,
							p_LossFunction,
							&map_intrinsics.at(view->id_intrinsic)[0],
							&map_poses.at(view->id_pose)[0],
							structure_landmark_it.second.X.data());
					}
					else
					{
						problem.AddResidualBlock(cost_function,
							p_LossFunction,
							&map_poses.at(view->id_pose)[0],
							structure_landmark_it.second.X.data());
					}
				}
				else
				{
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
		ceres_config_options.preconditioner_type =
			static_cast<ceres::PreconditionerType>(ceres_options_.preconditioner_type_);
		ceres_config_options.linear_solver_type =
			static_cast<ceres::LinearSolverType>(ceres_options_.linear_solver_type_);
		ceres_config_options.sparse_linear_algebra_library_type =
			static_cast<ceres::SparseLinearAlgebraLibraryType>(ceres_options_.sparse_linear_algebra_library_type_);
		ceres_config_options.minimizer_progress_to_stdout = ceres_options_.bVerbose_;
		ceres_config_options.logging_type = ceres::SILENT;
		ceres_config_options.num_threads = ceres_options_.nb_threads_;
		ceres_config_options.num_linear_solver_threads = ceres_options_.nb_threads_;
		ceres_config_options.parameter_tolerance = ceres_options_.parameter_tolerance_;

		// Solve BA
		ceres::Solver::Summary summary;
		ceres::Solve(ceres_config_options, &problem, &summary);
		if (ceres_options_.bCeres_summary_)
			std::cout << summary.FullReport() << std::endl;

		// If no error, get back refined parameters
		if (!summary.IsSolutionUsable())
		{
			if (ceres_options_.bVerbose_)
				std::cout << "Bundle Adjustment failed." << std::endl;
			return false;
		}
		else // Solution is usable
		{
			if (ceres_options_.bVerbose_)
			{
				// Display statistics about the minimization
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
			if (options.extrinsics_opt != Extrinsic_Parameter_Type::NONE)
			{
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
			
			// Structure is already updated directly if needed (no data wrapping)

			if (b_usable_prior)
			{
				// set back to the original scene centroid
				openMVG::sfm::ApplySimilarity(sim_to_center.inverse(), scene, true);

				//--
				// - Compute some fitting statistics
				//--

				// Collect corresponding camera centers
				std::vector<Vec3> X_SfM, X_GPS;
				for (const auto & view_it : scene.GetViews())
				{
					const sfm::ViewPriors * prior = dynamic_cast<sfm::ViewPriors*>(view_it.second.get());
					if (prior != nullptr && prior->b_use_pose_center_ && scene.IsPoseAndIntrinsicDefined(prior))
					{
						X_SfM.push_back(scene.GetPoses().at(prior->id_pose).center());
						X_GPS.push_back(prior->pose_center_);
					}
				}
				// Compute the registration fitting error (once BA with Prior have been used):
				if (X_GPS.size() > 3)
				{
					// Compute the median residual error
					Vec residual = (Eigen::Map<Mat3X>(X_SfM[0].data(), 3, X_SfM.size()) - Eigen::Map<Mat3X>(X_GPS[0].data(), 3, X_GPS.size())).colwise().norm();
					std::cout
						<< "Pose prior statistics (user units):\n"
						<< " - Starting median fitting error: " << pose_center_robust_fitting_error << "\n"
						<< " - Final fitting error:";
					minMaxMeanMedian<Vec::Scalar>(residual.data(), residual.data() + residual.size());
				}
			}
			return true;
	}

}