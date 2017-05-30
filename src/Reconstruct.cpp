
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


int main(int argc, char **argv)
{
	using namespace std;
	std::cout << "Sequential/Incremental reconstruction" << std::endl
		<< " Perform incremental SfM (Initial Pair Essential + Resection)." << std::endl
		<< std::endl;

	CmdLine cmd;

	std::string sSfM_Data_Filename = "D:\\Code\\test\\SfM_Data.json";
	std::string sMatchesDir = "D:\\Code\\test\\";
	std::string sOutDir = "D:\\Code\\test\\reconstruct";
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
