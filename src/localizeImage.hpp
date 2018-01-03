#pragma once

#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/image_describer_akaze_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "nonFree/sift/SIFT_describer_io.hpp"
#include "openMVG/sfm/sfm.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::sfm;

namespace coloc
{
	class Localizer {
	public:
		std::unique_ptr<features::Regions> regionsCurrent;
		Localizer(std::string& method);
		bool localizeImage(SfM_Data& scene, std::string& imageName, geometry::Pose3& pose);
		
	private:
		std::unique_ptr<features::Image_describer> image_describer;

		Mat3 K;

	};

	Localizer::Localizer(std::string& method)
	{
		if (method == "SIFT")
			image_describer.reset(new features::SIFT_Image_describer(features::SIFT_Image_describer::Params(), true));
		else if (method == "AKAZE")
			image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MLDB), true);


		K << 320, 0, 320,
			0, 320, 240,
			0, 0, 1;
	}

	bool Localizer::localizeImage(SfM_Data& scene, std::string& imageName, geometry::Pose3& pose)
	{
		if (scene.GetPoses().empty() || scene.GetLandmarks().empty())
		{
			std::cerr << "The input scene has no 3D content." << std::endl;
			return EXIT_FAILURE;
		}

		std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();

		using namespace openMVG::features;

		image::Image<unsigned char> imageGray;

		if (!ReadImage(imageName.c_str(), &imageGray)) {
			std::cout << "Unable to read image from the given path." << std::endl;
		}
		image_describer->Describe(imageGray, regionsCurrent);
		std::cout << "#regions detected in query image: " << regionsCurrent->RegionCount() << std::endl;

		std::string folderName;	
		C_Progress_display progress;

		std::unique_ptr<Regions> regions_type;
		regions_type.reset(new features::AKAZE_Binary_Regions());
		
		std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();
		if (!regions_provider->load(scene, folderName, regions_type, &progress)) {
			std::cerr << std::endl << "Invalid regions." << std::endl;
			return EXIT_FAILURE;
		}

		sfm::SfM_Localization_Single_3DTrackObservation_Database localizer;
		if (!localizer.Init(scene, *regions_provider.get()))
		{
			std::cerr << "Cannot initialize the SfM localizer" << std::endl;
		}
		// Since we have copied interesting data, release some memory
		regions_provider.reset();

		sfm::Image_Localizer_Match_Data matching_data;
		matching_data.error_max = 0.2;

		bool bSuccessfulLocalization = false;

		const openMVG::cameras::Pinhole_Intrinsic
			camL(640, 480, K(0, 0), K(0, 2), K(1, 2));

		// Try to localize the image in the database thanks to its regions
		if (!localizer.Localize(
			resection::SolverType::P3P_KE_CVPR17,
			{ imageGray.Width(), imageGray.Height() },
			&camL,
			*(regionsCurrent.get()),
			pose,
			&matching_data))
			std::cout << "Localization successful" << std::endl;
	}
}
