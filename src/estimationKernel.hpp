#pragma once

// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

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



namespace coloc
{/*
	template <typename SolverArg, typename ModelArg = Mat34>
		class ACRKernel
	{
	public:
		using Solver = SolverArg;
		using Model = ModelArg;

		ACRKernel
		(
			const Mat & x2d, // Undistorted 2d feature_point location
			const Mat & x3D, // 3D corresponding points
			const cameras::IntrinsicBase * camera
		) :x2d_(x2d),
			x3D_(x3D),
			logalpha0_(log10(M_PI)),
			N1_(Mat3::Identity()),
			camera_(camera)
		{
			N1_.diagonal().head(2) *= camera->imagePlane_toCameraPlaneError(1.0);
			assert(2 == x2d_.rows());
			assert(3 == x3D_.rows());
			assert(x2d_.cols() == x3D_.cols());
			bearing_vectors_ = camera->operator()(x2d_);
		}

		enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
		enum { MAX_MODELS = Solver::MAX_MODELS };

		void Fit(const std::vector<uint32_t> &samples, std::vector<Model> *models) const {
			Solver::Solve(ExtractColumns(bearing_vectors_, samples), // bearing vectors
				ExtractColumns(x3D_, samples), // 3D points
				models); // Found model hypothesis
		}

		double Error(uint32_t sample, const Model &model) const 
		{			
			const Vec3 t = model.block(0, 3, 3, 1);
			const geometry::Pose3 pose(model.block(0, 0, 3, 3),
				-model.block(0, 0, 3, 3).transpose() * t);
			const bool ignore_distortion = true; // We ignore distortion since we are using undistorted bearing vector as input
			return (camera_->residual(pose(x3D_.col(sample)),
				x2d_.col(sample),
				ignore_distortion) * N1_(0, 0)).squaredNorm();
		}

		void Errors(const Model & model, std::vector<double> & vec_errors) const
		{
			const Vec3 t = model.block(0, 3, 3, 1);
			const geometry::Pose3 pose(model.block(0, 0, 3, 3),
				-model.block(0, 0, 3, 3).transpose() * t);

			vec_errors.resize(x2d_.cols());
			for (Mat::Index sample = 0; sample < x2d_.cols(); ++sample)
				vec_errors[sample] = this->Error(sample, model);
		}

		size_t NumSamples() const { return x2d_.cols(); }

		void Unnormalize(Model * model) const {	}

		double logalpha0() const { return logalpha0_; }
		double multError() const { return 1.0; } // point to point error
		Mat3 normalizer1() const { return Mat3::Identity(); }
		Mat3 normalizer2() const { return N1_; }
		double unormalizeError(double val) const { return sqrt(val) / N1_(0, 0); }

	private:
		Mat x2d_, bearing_vectors_;
		const Mat & x3D_;
		Mat3 N1_;
		double logalpha0_;  // Alpha0 is used to make the error adaptive to the image size
		const cameras::IntrinsicBase * camera_;   // Intrinsic camera parameter
	};

*/
		bool filterHomography(
			LocalizationParams& params,
			Regions& regions1, Regions& regions2,
			const Pair pairIndex,
			const openMVG::matching::IndMatches & vec_PutativeMatches,
			matching::IndMatches & geometric_inliers)
		{
			using namespace openMVG;
			using namespace openMVG::robust;
			geometric_inliers.clear();

			// Get back corresponding view index
			const IndexT iIndex = pairIndex.first;
			const IndexT jIndex = pairIndex.second;

			//--
			// Get corresponding point regions arrays
			//--

			Mat2X xI, xJ;
			//matching_image_collection::MatchesPairToMat(pairIndex, vec_PutativeMatches, sfm_data, regions_provider, xI, xJ);

			const Pinhole_Intrinsic
				camL(params.imageSize.first, params.imageSize.second, (params.K)(0, 0), (params.K)(0, 2), (params.K)(1, 2)),
				camR(params.imageSize.first, params.imageSize.second, (params.K)(0, 0), (params.K)(0, 2), (params.K)(1, 2));

			const features::PointFeatures
				feature_I = regions1.GetRegionsPositions(),
				feature_J = regions2.GetRegionsPositions();
			matching_image_collection::MatchesPointsToMat(
				vec_PutativeMatches,
				&camL, feature_I,
				&camR, feature_J,
				xI, xJ);
			//--
			// Robust estimation
			//--

			// Define the AContrario adapted Homography matrix solver
			using KernelType =
				ACKernelAdaptor<
				openMVG::homography::kernel::FourPointSolver,
				openMVG::homography::kernel::AsymmetricError,
				UnnormalizerI,
				Mat3>;

			KernelType kernel(
				xI, params.imageSize.first, params.imageSize.second,
				xJ, params.imageSize.first, params.imageSize.second,
				false); // configure as point to point error model.

						// Robustly estimate the Homography matrix with A Contrario ransac
			const double upper_bound_precision = Square(std::numeric_limits<double>::infinity());
			std::vector<uint32_t> vec_inliers;
			Mat3 H = Mat3::Identity();
			const std::pair<double, double> ACRansacOut =
				ACRANSAC(kernel, vec_inliers, 1024, &H, upper_bound_precision);

			if (vec_inliers.size() > KernelType::MINIMUM_SAMPLES *2.5) {
				double m_dPrecision_robust = ACRansacOut.first;
				// update geometric_inliers
				geometric_inliers.reserve(vec_inliers.size());
				for (const uint32_t & index : vec_inliers) {
					geometric_inliers.push_back(vec_PutativeMatches[index]);
				}
				return true;
			}
			else {
				vec_inliers.clear();
				return false;
			}
			return true;
		}
/*
	bool robustRelativePose
	(
		const IntrinsicBase * intrinsics1,
		const IntrinsicBase * intrinsics2,
		const Mat & x1,
		const Mat & x2,
		RelativePose_Info & relativePose_info,
		const std::pair<size_t, size_t> & size_ima1,
		const std::pair<size_t, size_t> & size_ima2,
		const size_t max_iteration_count
	)
	{
		if (!intrinsics1 || !intrinsics2)
			return false;

		// Compute the bearing vectors
		const Mat3X
			bearing1 = (*intrinsics1)(x1),
			bearing2 = (*intrinsics2)(x2);

		if (isPinhole(intrinsics1->getType()) && isPinhole(intrinsics2->getType()))
		{
			using KernelType = robust::ACKernelAdaptorEssential<
				openMVG::essential::kernel::FivePointSolver,
				openMVG::fundamental::kernel::EpipolarDistanceError,
				Mat3>;
			KernelType kernel(x1, bearing1, size_ima1.first, size_ima1.second,
				x2, bearing2, size_ima2.first, size_ima2.second,
				dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics1)->K(),
				dynamic_cast<const cameras::Pinhole_Intrinsic*>(intrinsics2)->K());

			// Robustly estimation of the Model and its precision
			const auto ac_ransac_output = robust::ACRANSAC(
				kernel, relativePose_info.vec_inliers,
				max_iteration_count, &relativePose_info.essential_matrix,
				relativePose_info.initial_residual_tolerance, false);

			relativePose_info.found_residual_precision = ac_ransac_output.first;

			if (relativePose_info.vec_inliers.size() < 2.5 * KernelType::Solver::MINIMUM_SAMPLES)
				return false; // no sufficient coverage (the model does not support enough samples)

		}

		// estimation of the relative poses based on the cheirality test
		Pose3 relative_pose;
		if (!RelativePoseFromEssential(
			bearing1,
			bearing2,
			relativePose_info.essential_matrix,
			relativePose_info.vec_inliers, &relative_pose))
		{
			return false;
		}
		relativePose_info.relativePose = relative_pose;
		return true;
	}



	bool Localize
	(
		const resection::SolverType & solver_type,
		const Pair & image_size,
		const cameras::IntrinsicBase * optional_intrinsics,
		Image_Localizer_Match_Data & resection_data,
		geometry::Pose3 & pose
	)
	{
		Mat34 P;
		resection_data.vec_inliers.clear();

		// Setup the admissible upper bound residual error
		const double dPrecision = resection_data.error_max == std::numeric_limits<double>::infinity() ?
			std::numeric_limits<double>::infinity() : Square(resection_data.error_max);

		size_t MINIMUM_SAMPLES = 0;

		using SolverType = openMVG::euclidean_resection::P3PSolver_Ke;
		MINIMUM_SAMPLES = SolverType::MINIMUM_SAMPLES;
		
		switch (solver_type) {
			case resection::SolverType::P3P_KE_CVPR17:
			{
				using SolverType = openMVG::euclidean_resection::P3PSolver_Ke;
				MINIMUM_SAMPLES = SolverType::MINIMUM_SAMPLES;
			}
			break;

			case resection::SolverType::P3P_KNEIP_CVPR11:
			{
				using SolverType = openMVG::euclidean_resection::P3PSolver_Kneip;
				MINIMUM_SAMPLES = SolverType::MINIMUM_SAMPLES;
			}
			break;
		
		}

		if (!optional_intrinsics) {
			std::cerr << "Intrinsic data is required for P3P solvers." << std::endl;
			return false;
		}

		using KernelType = ACRKernel<SolverType, Mat34>;

		KernelType kernel(resection_data.pt2D, resection_data.pt3D, optional_intrinsics);
		// Robust estimation of the pose matrix and its precision
		const auto ACRansacOut = openMVG::robust::ACRANSAC(kernel, resection_data.vec_inliers, resection_data.max_iteration,
				&P, dPrecision,	true);

		// Update the upper bound precision of the model found by AC-RANSAC
		resection_data.error_max = ACRansacOut.first;

		// Test if the mode support some points (more than those required for estimation)
		const bool bResection = (resection_data.vec_inliers.size() > 2.5 * MINIMUM_SAMPLES);

		if (bResection)
		{
			resection_data.projection_matrix = P;
			Mat3 K, R;
			Vec3 t;
			KRt_From_P(P, &K, &R, &t);
			pose = geometry::Pose3(R, -R.transpose() * t);
		}

		std::cout
			<< "Pose estimation:" << "\n"
			<< "Status: " << (bResection ? "Success" : "Failure") << std::endl
			<< "Feature points tracked: " << resection_data.pt2D.cols() << std::endl
			<< "Feature points validated: " << resection_data.vec_inliers.size() << std::endl
			<< "Threshold: " << resection_data.error_max << std::endl;

		return bResection;
	}
	*/
}
