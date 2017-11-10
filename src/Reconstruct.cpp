
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <cstdlib>

#include "deps/Localizer.hpp"
#include "deps/BundleAdjustmentCeres.hpp"
#include "deps/SequentialReconstruction.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"

using namespace openMVG;

/// From 2 given image file-names, find the two corresponding index in the View list
bool computeIndexFromImageNames(
	const sfm::SfM_Data & SfM_Data,
	const std::pair<std::string, std::string>& initialPairName,
	Pair& initialPairIndex)
{
	if (initialPairName.first == initialPairName.second)
	{
		std::cerr << "\nInvalid image names. You cannot use the same image to initialize a pair." << std::endl;
		return false;
	}

	initialPairIndex = Pair(UndefinedIndexT, UndefinedIndexT);

	/// List views filenames and find the one that correspond to the user ones:
	for (sfm::Views::const_iterator it = SfM_Data.GetViews().begin();
		it != SfM_Data.GetViews().end(); ++it)
	{
		const sfm::View * v = it->second.get();
		const std::string filename = stlplus::filename_part(v->s_Img_path);
		if (filename == initialPairName.first)
		{
			initialPairIndex.first = v->id_view;
		}
		else {
			if (filename == initialPairName.second)
			{
				initialPairIndex.second = v->id_view;
			}
		}
	}
	return (initialPairIndex.first != UndefinedIndexT &&
		initialPairIndex.second != UndefinedIndexT);
}

bool AutomaticInitialPairChoice(Pair & initial_pair)
{
	// select a pair that have the largest baseline (mean angle between its bearing vectors).

	const unsigned iMin_inliers_count = 100;
	const float fRequired_min_angle = 3.0f;
	const float fLimit_max_angle = 60.0f; // More than 60 degree, we cannot rely on matches for initial pair seeding

										  // List Views that support valid intrinsic (view that could be used for Essential matrix computation)
	std::set<IndexT> valid_views;
	for (Views::const_iterator it = sfm_data_.GetViews().begin();
		it != sfm_data_.GetViews().end(); ++it)
	{
		const View * v = it->second.get();
		if (sfm_data_.GetIntrinsics().count(v->id_intrinsic))
			valid_views.insert(v->id_view);
	}

	if (valid_views.size() < 2)
	{
		return false; // There is not view that support valid intrinsic data
	}

	std::vector<std::pair<double, Pair> > scoring_per_pair;

	// Compute the relative pose & the 'baseline score'
	C_Progress_display my_progress_bar(matches_provider_->pairWise_matches_.size(),
		std::cout,
		"Automatic selection of an initial pair:\n");
#ifdef OPENMVG_USE_OPENMP
#pragma omp parallel
#endif
	for (const std::pair< Pair, IndMatches > & match_pair : matches_provider_->pairWise_matches_)
	{
#ifdef OPENMVG_USE_OPENMP
#pragma omp single nowait
#endif
		{
			++my_progress_bar;

			const Pair current_pair = match_pair.first;

			const uint32_t I = std::min(current_pair.first, current_pair.second);
			const uint32_t J = std::max(current_pair.first, current_pair.second);
			if (valid_views.count(I) && valid_views.count(J))
			{
				const View * view_I = sfm_data_.GetViews().at(I).get();
				const Intrinsics::const_iterator iterIntrinsic_I = sfm_data_.GetIntrinsics().find(view_I->id_intrinsic);
				const View * view_J = sfm_data_.GetViews().at(J).get();
				const Intrinsics::const_iterator iterIntrinsic_J = sfm_data_.GetIntrinsics().find(view_J->id_intrinsic);

				const Pinhole_Intrinsic * cam_I = dynamic_cast<const Pinhole_Intrinsic*>(iterIntrinsic_I->second.get());
				const Pinhole_Intrinsic * cam_J = dynamic_cast<const Pinhole_Intrinsic*>(iterIntrinsic_J->second.get());
				if (cam_I != nullptr && cam_J != nullptr)
				{
					openMVG::tracks::STLMAPTracks map_tracksCommon;
					shared_track_visibility_helper_->GetTracksInImages({ I, J }, map_tracksCommon);

					// Copy points correspondences to arrays for relative pose estimation
					const size_t n = map_tracksCommon.size();
					Mat xI(2, n), xJ(2, n);
					size_t cptIndex = 0;
					for (openMVG::tracks::STLMAPTracks::const_iterator
						iterT = map_tracksCommon.begin(); iterT != map_tracksCommon.end();
						++iterT, ++cptIndex)
					{
						tracks::submapTrack::const_iterator iter = iterT->second.begin();
						const uint32_t i = iter->second;
						const uint32_t j = (++iter)->second;

						Vec2 feat = features_provider_->feats_per_view[I][i].coords().cast<double>();
						xI.col(cptIndex) = cam_I->get_ud_pixel(feat);
						feat = features_provider_->feats_per_view[J][j].coords().cast<double>();
						xJ.col(cptIndex) = cam_J->get_ud_pixel(feat);
					}

					// Robust estimation of the relative pose
					RelativePose_Info relativePose_info;
					relativePose_info.initial_residual_tolerance = Square(4.0);

					if (robustRelativePose(
						cam_I->K(), cam_J->K(),
						xI, xJ, relativePose_info,
						{ cam_I->w(), cam_I->h() }, { cam_J->w(), cam_J->h() },
						256) && relativePose_info.vec_inliers.size() > iMin_inliers_count)
					{
						// Triangulate inliers & compute angle between bearing vectors
						std::vector<float> vec_angles;
						vec_angles.reserve(relativePose_info.vec_inliers.size());
						const Pose3 pose_I = Pose3(Mat3::Identity(), Vec3::Zero());
						const Pose3 pose_J = relativePose_info.relativePose;
						const Mat34 PI = cam_I->get_projective_equivalent(pose_I);
						const Mat34 PJ = cam_J->get_projective_equivalent(pose_J);
						for (const uint32_t & inlier_idx : relativePose_info.vec_inliers)
						{
							Vec3 X;
							TriangulateDLT(
								PI, xI.col(inlier_idx).homogeneous(),
								PJ, xJ.col(inlier_idx).homogeneous(), &X);

							openMVG::tracks::STLMAPTracks::const_iterator iterT = map_tracksCommon.begin();
							std::advance(iterT, inlier_idx);
							tracks::submapTrack::const_iterator iter = iterT->second.begin();
							const Vec2 featI = features_provider_->feats_per_view[I][iter->second].coords().cast<double>();
							const Vec2 featJ = features_provider_->feats_per_view[J][(++iter)->second].coords().cast<double>();
							vec_angles.push_back(AngleBetweenRay(pose_I, cam_I, pose_J, cam_J, featI, featJ));
						}
						// Compute the median triangulation angle
						const unsigned median_index = vec_angles.size() / 2;
						std::nth_element(
							vec_angles.begin(),
							vec_angles.begin() + median_index,
							vec_angles.end());
						const float scoring_angle = vec_angles[median_index];
						// Store the pair iff the pair is in the asked angle range [fRequired_min_angle;fLimit_max_angle]
						if (scoring_angle > fRequired_min_angle &&
							scoring_angle < fLimit_max_angle)
						{
#ifdef OPENMVG_USE_OPENMP
#pragma omp critical
#endif
							scoring_per_pair.emplace_back(scoring_angle, current_pair);
						}
					}
				}
			}
		} // omp section
	}
	std::sort(scoring_per_pair.begin(), scoring_per_pair.end());
	// Since scoring is ordered in increasing order, reverse the order
	std::reverse(scoring_per_pair.begin(), scoring_per_pair.end());
	if (!scoring_per_pair.empty())
	{
		initial_pair = scoring_per_pair.begin()->second;
		return true;
	}
	return false;
}


int main(int argc, char **argv)
{
	using namespace std;
	std::cout << "Sequential/Incremental reconstruction" << std::endl
		<< " Perform incremental SfM (Initial Pair Essential + Resection)." << std::endl
		<< std::endl;

	CmdLine cmd;

	std::string sSfM_Data_Filename = "C:\\Users\\svempral\\Desktop\\openMVGtest\\sfm_data.json";
	std::string sMatchesDir = "C:\\Users\\svempral\\Desktop\\openMVGtest\\sfm_data.json";
	std::string sOutDir = "C:\\Users\\svempral\\Desktop\\openMVGtest\\reconstructed";
	std::pair<std::string, std::string> initialPairString("", "");
	std::string sIntrinsic_refinement_options = "ADJUST_ALL";
	int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;
	bool b_use_motion_priors = false;

	cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
	cmd.add(make_option('m', sMatchesDir, "matchdir"));
	cmd.add(make_option('o', sOutDir, "outdir"));
	cmd.add(make_option('a', initialPairString.first, "initialPairA"));
	cmd.add(make_option('b', initialPairString.second, "initialPairB"));
	cmd.add(make_option('c', i_User_camera_model, "camera_model"));
	cmd.add(make_option('f', sIntrinsic_refinement_options, "refineIntrinsics"));
	cmd.add(make_switch('P', "prior_usage"));

	try {
		//if (argc == 1) throw std::string("Invalid parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Usage: " << argv[0] << '\n'
			<< "[-i|--input_file] path to a SfM_Data scene\n"
			<< "[-m|--matchdir] path to the matches that corresponds to the provided SfM_Data scene\n"
			<< "[-o|--outdir] path where the output data will be stored\n"
			<< "\n[Optional]\n"
			<< "[-a|--initialPairA] filename of the first image (without path)\n"
			<< "[-b|--initialPairB] filename of the second image (without path)\n"
			<< "[-c|--camera_model] Camera model type for view with unknown intrinsic:\n"
			<< "\t 1: Pinhole \n"
			<< "\t 2: Pinhole radial 1\n"
			<< "\t 3: Pinhole radial 3 (default)\n"
			<< "\t 4: Pinhole radial 3 + tangential 2\n"
			<< "\t 5: Pinhole fisheye\n"
			<< "[-f|--refineIntrinsics] Intrinsic parameters refinement option\n"
			<< "\t ADJUST_ALL -> refine all existing parameters (default) \n"
			<< "\t NONE -> intrinsic parameters are held as constant\n"
			<< "\t ADJUST_FOCAL_LENGTH -> refine only the focal length\n"
			<< "\t ADJUST_PRINCIPAL_POINT -> refine only the principal point position\n"
			<< "\t ADJUST_DISTORTION -> refine only the distortion coefficient(s) (if any)\n"
			<< "\t -> NOTE: options can be combined thanks to '|'\n"
			<< "\t ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT\n"
			<< "\t\t-> refine the focal length & the principal point position\n"
			<< "\t ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION\n"
			<< "\t\t-> refine the focal length & the distortion coefficient(s) (if any)\n"
			<< "\t ADJUST_PRINCIPAL_POINT|ADJUST_DISTORTION\n"
			<< "\t\t-> refine the principal point position & the distortion coefficient(s) (if any)\n"
			<< "[-P|--prior_usage] Enable usage of motion priors (i.e GPS positions) (default: false)\n"
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}

	if (i_User_camera_model < PINHOLE_CAMERA ||
		i_User_camera_model > PINHOLE_CAMERA_FISHEYE) {
		std::cerr << "\n Invalid camera type" << std::endl;
		return EXIT_FAILURE;
	}

	const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
		cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);
	if (intrinsic_refinement_options == static_cast<cameras::Intrinsic_Parameter_Type>(0))
	{
		std::cerr << "Invalid input for Bundle Adjusment Intrinsic parameter refinement option" << std::endl;
		return EXIT_FAILURE;
	}

	// Load input SfM_Data scene
	sfm::SfM_Data SfM_Data;
	if (!sfm::Load(SfM_Data, sSfM_Data_Filename, sfm::ESfM_Data(sfm::VIEWS | sfm::INTRINSICS))) {
		std::cerr << std::endl
			<< "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
		return EXIT_FAILURE;
	}

	// Init the regions_type from the image describer file (used for image regions extraction)
	using namespace openMVG::features;
	const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
	std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
	if (!regions_type)
	{
		std::cerr << "Invalid: "
			<< sImage_describer << " regions type file." << std::endl;
		return EXIT_FAILURE;
	}

	// Features reading
	std::shared_ptr<sfm::Features_Provider> feats_provider = std::make_shared<sfm::Features_Provider>();
	if (!feats_provider->load(SfM_Data, sMatchesDir, regions_type)) {
		std::cerr << std::endl
			<< "Invalid features." << std::endl;
		return EXIT_FAILURE;
	}
	// Matches reading
	std::shared_ptr<sfm::Matches_Provider> MProviderStruct = std::make_shared<sfm::Matches_Provider>();
	if // Try to read the two matches file formats
		(
			!(MProviderStruct->load(SfM_Data, stlplus::create_filespec(sMatchesDir, "matches.f.txt")) ||
				MProviderStruct->load(SfM_Data, stlplus::create_filespec(sMatchesDir, "matches.f.bin")))
			)
	{
		std::cerr << std::endl
			<< "Invalid matches file." << std::endl;
		return EXIT_FAILURE;
	}

	if (sOutDir.empty()) {
		std::cerr << "\nIt is an invalid output directory" << std::endl;
		return EXIT_FAILURE;
	}

	if (!stlplus::folder_exists(sOutDir))
	{
		if (!stlplus::folder_create(sOutDir))
		{
			std::cerr << "\nCannot create the output directory" << std::endl;
		}
	}

	//---------------------------------------
	// Sequential reconstruction process
	//---------------------------------------

	openMVG::system::Timer timer;
	Reconstruction sfmEngine(
		SfM_Data,
		sOutDir,
		stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));

	// Configure the FProviderStruct & the MProviderStruct
	sfmEngine.SetFeaturesProvider(feats_provider.get());
	sfmEngine.SetMatchesProvider(MProviderStruct.get());

	// Configure reconstruction parameters
	sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
	sfmEngine.SetUnknownCameraType(EINTRINSIC(i_User_camera_model));
	b_use_motion_priors = cmd.used('P');
	sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);

	// Handle Initial pair parameter
	if (!initialPairString.first.empty() && !initialPairString.second.empty())
	{
		Pair initialPairIndex;
		if (!computeIndexFromImageNames(SfM_Data, initialPairString, initialPairIndex))
		{
			std::cerr << "Could not find the initial pairs <" << initialPairString.first
				<< ", " << initialPairString.second << ">!\n";
			return EXIT_FAILURE;
		}
		sfmEngine.setInitialPair(initialPairIndex);
	}

	if (!AutomaticInitialPairChoice(initial_pair_))
	{
		// Cannot find a valid initial pair, try to set it by hand?
		if (!ChooseInitialPair(initial_pair_))
		{
			return false;
		}
	}



	if (sfmEngine.Process())
	{
		std::cout << std::endl << " Total Ac-Sfm took (s): " << timer.elapsed() << std::endl;

		std::cout << "...Generating SfM_Report.html" << std::endl;
		sfm::Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
			stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));

		//-- Export to disk computed scene (data & visualizable results)
		std::cout << "...Export SfM_Data to disk." << std::endl;
		Save(sfmEngine.Get_SfM_Data(),
			stlplus::create_filespec(sOutDir, "SfM_Data", ".bin"),
			sfm::ESfM_Data(sfm::ALL));

		Save(sfmEngine.Get_SfM_Data(),
			stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
			sfm::ESfM_Data(sfm::ALL));

		return EXIT_SUCCESS;
	}
	return EXIT_FAILURE;
}
