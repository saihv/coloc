// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// The <cereal/archives> headers are special and must be included first.
#include <cereal/archives/json.hpp>

#include <openMVG/sfm/sfm.hpp>
#include <openMVG/features/feature.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/image/image_io.hpp>
#include "software/SfM/SfMPlyHelper.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/multiview/solver_resection_kernel.hpp"
#include "openMVG/multiview/solver_resection_p3p.hpp"

#include <openMVG/system/timer.hpp>
#include "openMVG/stl/stl.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::matching;

#include "nonFree/sift/SIFT_describer_io.hpp"
#include "openMVG/features/image_describer_akaze_io.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

// Naive function for finding the biggest common root dir from two paths
std::string FindCommonRootDir(const std::string & dir1, const std::string & dir2)
{
	int i = 0;
	for (; i != std::min(dir1.size(), dir2.size()); i++)
	{
		if (dir1[i] != dir2[i]) break;
	}
	return dir1.substr(0, i);
}

// ----------------------------------------------------
// Multiple Images localization from an existing reconstruction
// ----------------------------------------------------
int main(int argc, char **argv)
{
	SfM_Data sfm_data;
	std::string sSfM_Data_Filename = "C:\\Users\\svempral\\Desktop\\openMVGtest\\sfm_data.bin";
	std::string sMatchesDir = "C:\\Users\\svempral\\Desktop\\openMVGtest";
	if (sfm_data.GetPoses().empty() || sfm_data.GetLandmarks().empty())
	{
		std::cerr << std::endl
			<< "The input SfM_Data file have not 3D content to match with." << std::endl;
		return EXIT_FAILURE;
	}

	using namespace openMVG::features;
	const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
	std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
	if (!regions_type)
	{
		std::cerr << "Invalid: "
			<< sImage_describer << " regions type file." << std::endl;
		return EXIT_FAILURE;
	}
	C_Progress_display progress;
	// Load the SfM_Data region's views
	std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();
	if (!regions_provider->load(sfm_data, sMatchesDir, regions_type, &progress)) {
		std::cerr << std::endl << "Invalid regions." << std::endl;
		return EXIT_FAILURE;
	}

	if (sfm_data.GetPoses().empty() || sfm_data.GetLandmarks().empty())
	{
		std::cerr << std::endl
			<< "The input SfM_Data file have not 3D content to match with." << std::endl;
		return false;
	}

	// Setup the database
	// A collection of regions
	// - each view observation leads to a new regions
	// - link each observation region to a track id to ease 2D-3D correspondences search

	std::unique_ptr<features::Regions> landmark_observations_descriptors_;
	/// Association of a track observation to a track Id (used for retrieval)
	std::vector<IndexT> index_to_landmark_id_;
	/// A matching interface to find matches between 2D descriptor matches
	///  and 3D points observation descriptors
	std::shared_ptr<matching::Matcher_Regions_Database> matching_interface_;

	openMVG::sfm::Regions_Provider& rp = *regions_provider.get();

	landmark_observations_descriptors_.reset(rp.getRegionsType()->EmptyClone());
	for (const auto & landmark : sfm_data.GetLandmarks())
	{
		for (const auto & observation : landmark.second.obs)
		{
			if (observation.second.id_feat != UndefinedIndexT)
			{
				// copy the feature/descriptor to landmark_observations_descriptors
				const std::shared_ptr<features::Regions> view_regions = rp.get(observation.first);
				view_regions->CopyRegion(observation.second.id_feat, landmark_observations_descriptors_.get());
				// link this descriptor to the track Id
				index_to_landmark_id_.push_back(landmark.first);
			}
		}
	}
	std::cout << "Init retrieval database ... " << std::endl;
	//matching_interface_.reset(new
//		matching::Matcher_Regions_Database(matching::BRUTE_FORCE_HAMMING, *landmark_observations_descriptors_));
	//std::cout << "Retrieval database initialized with:\n"
	//	<< "#landmarks: " << sfm_data.GetLandmarks().size() << "\n"
	//	<< "#descriptors: " << landmark_observations_descriptors_->RegionCount() << std::endl;

	std::unique_ptr<Regions> query_regions(regions_type->EmptyClone());

	const std::string sView_filename = stlplus::create_filespec(sQueryDir, *iter_image);
	// Try to open image
	if (!image::ReadImage(sView_filename.c_str(), &imageGray))
	{
		std::cerr << "Cannot open the input provided image : " << *iter_image << std::endl;
		continue;
	}

	const std::string
		sFeat = stlplus::create_filespec(sMatchesOutDir, stlplus::basename_part(sView_filename.c_str()), "feat"),
		sDesc = stlplus::create_filespec(sMatchesOutDir, stlplus::basename_part(sView_filename.c_str()), "desc");

	// Compute features and descriptors and save them if they don't exist yet
	if (!stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc))
	{
		image_describer->Describe(imageGray, query_regions);
		image_describer->Save(query_regions.get(), sFeat, sDesc);
		std::cout << "#regions detected in query image: " << query_regions->RegionCount() << std::endl;
	}

	std::vector<IndMatch> vec_PutativeMatches;
	
	matching::DistanceRatioMatch(0.8, matching::BRUTE_FORCE_HAMMING, *landmark_observations_descriptors_.get(), *query_regions.get(), vec_PutativeMatches);

	std::cout << "#3D2d putative correspondences: " << vec_putative_matches.size() << std::endl;


	Image_Localizer_Match_Data resection_data;
	/*
	if (resection_data_ptr)
	{
		resection_data.error_max = resection_data_ptr->error_max;
	}
	*/
	resection_data.pt3D.resize(3, vec_putative_matches.size());
	resection_data.pt2D.resize(2, vec_putative_matches.size());
	Mat2X pt2D_original(2, vec_putative_matches.size());
	for (size_t i = 0; i < vec_putative_matches.size(); ++i)
	{
		resection_data.pt3D.col(i) = sfm_data.GetLandmarks().at(index_to_landmark_id_[vec_putative_matches[i].i_]).X;
		resection_data.pt2D.col(i) = query_regions->GetRegionPosition(vec_putative_matches[i].j_);
		pt2D_original.col(i) = resection_data.pt2D.col(i);
		// Handle image distortion if intrinsic is known (to ease the resection)
		if (optional_intrinsics && optional_intrinsics->have_disto())
		{
			resection_data.pt2D.col(i) = optional_intrinsics->get_ud_pixel(resection_data.pt2D.col(i));
		}
	}

	Pair imageSize = { imageGray.Width(), imageGray.Height() };

	Mat34 P;
	resection_data.vec_inliers.clear();

	// Setup the admissible upper bound residual error
	const double dPrecision =
		resection_data.error_max == std::numeric_limits<double>::infinity() ?
		std::numeric_limits<double>::infinity() :
		Square(resection_data.error_max);

	size_t MINIMUM_SAMPLES = 0;
	const cameras::Pinhole_Intrinsic * pinhole_cam =
		dynamic_cast<const cameras::Pinhole_Intrinsic *>(optional_intrinsics);


	if (pinhole_cam == nullptr)
	{
		std::cerr << "Intrinsic data is required for P3P solvers." << std::endl;
		return false;
	}
	//--
	// Since K calibration matrix is known, compute only [R|t]
	using SolverType = openMVG::euclidean_resection::P3PSolver_Ke;
	MINIMUM_SAMPLES = SolverType::MINIMUM_SAMPLES;

	using KernelType =
		openMVG::robust::ACKernelAdaptorResection_K<
		SolverType,
		ResectionSquaredResidualError,
		Mat34>;

	KernelType kernel(resection_data.pt2D, resection_data.pt3D, pinhole_cam->K());
	// Robust estimation of the Projection matrix and it's precision
	const std::pair<double, double> ACRansacOut =
		openMVG::robust::ACRANSAC(kernel,
			resection_data.vec_inliers,
			resection_data.max_iteration,
			&P,
			dPrecision,
			true);
	// Update the upper bound precision of the model found by AC-RANSAC
	resection_data.error_max = ACRansacOut.first;

	const bool bResection = (resection_data.vec_inliers.size() > 2.5 * MINIMUM_SAMPLES);

	openMVG::geometry::Pose3 pose;

	if (bResection)
	{
		resection_data.projection_matrix = P;
		Mat3 K, R;
		Vec3 t;
		KRt_From_P(P, &K, &R, &t);
		pose = geometry::Pose3(R, -R.transpose() * t);
	}

	std::cout << "\n"
		<< "-------------------------------" << "\n"
		<< "-- Robust Resection " << "\n"
		<< "-- Resection status: " << bResection << "\n"
		<< "-- #Points used for Resection: " << resection_data.pt2D.cols() << "\n"
		<< "-- #Points validated by robust Resection: " << resection_data.vec_inliers.size() << "\n"
		<< "-- Threshold: " << resection_data.error_max << "\n"
		<< "-------------------------------" << std::endl;


	return EXIT_SUCCESS;
}
