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
		Localizer(std::string& method, std::pair <size_t, size_t> & size, Mat3& intrinsicMatrix)
		{
			if (method == "SIFT")
				image_describer.reset(new features::SIFT_Image_describer(features::SIFT_Image_describer::Params(), true));
			else if (method == "AKAZE")
				image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MLDB), true);

			this->imageSize = &size;
			this->K = &intrinsicMatrix;
		}

		std::unique_ptr<features::Regions> regionsCurrent;
		bool localizeImage(std::string& imageName, geometry::Pose3& pose);
		bool setupMap(LocalizationData&);
		
	private:
		std::unique_ptr<features::Image_describer> image_describer;
		std::pair <size_t, size_t> *imageSize;
		Mat3 *K;

		std::unique_ptr<Regions> regions_type;
		std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();
		sfm::SfM_Localization_Single_3DTrackObservation_Database localizer;
	};

	bool Localizer::setupMap(LocalizationData& data)
	{
		if (data.scene.GetPoses().empty() || data.scene.GetLandmarks().empty()) {
			std::cerr << "The input scene has no 3D content." << std::endl;
			return EXIT_FAILURE;
		}

		std::string folderName = "C:/Users/saihv/Desktop/test";
		C_Progress_display progress;

		std::unique_ptr<Regions> regions_type;
		regions_type.reset(new features::AKAZE_Binary_Regions());

		regions_provider->load(data.scene, folderName, regions_type, &progress);


		if (!this->localizer.Init(data.scene, *regions_provider.get())) {
			std::cerr << "Cannot initialize the SfM localizer" << std::endl;
		}
		else
			std::cout << "Initialized SFM localizer with map" << std::endl;
	}

	bool Localizer::localizeImage(std::string& imageName, geometry::Pose3& pose)
	{

		using namespace openMVG::features;

		image::Image<unsigned char> imageGray;

		if (!ReadImage(imageName.c_str(), &imageGray)) {
			std::cout << "Unable to read image from the given path." << std::endl;
		}
		image_describer->Describe(imageGray, regionsCurrent);
		std::cout << "#regions detected in query image: " << regionsCurrent->RegionCount() << std::endl;

		
		// Since we have copied interesting data, release some memory
		regions_provider.reset();

		sfm::Image_Localizer_Match_Data matching_data;
		matching_data.error_max = 0.2;

		bool bSuccessfulLocalization = false;

		const openMVG::cameras::Pinhole_Intrinsic
			camL(imageSize->first, imageSize->second, (*K)(0, 0), (*K)(0, 2), (*K)(1, 2));

		// Try to localize the image in the database thanks to its regions
		if (!localizer.Localize(
			resection::SolverType::P3P_KE_CVPR17,
			{ imageGray.Width(), imageGray.Height() },
			&camL,
			*(regionsCurrent.get()),
			pose,
			&matching_data)) {
			std::cout << "Localization successful" << std::endl;
			return EXIT_SUCCESS;
		}
		else {
			std::cout << "Cannot localize image" << std::endl;
			return EXIT_FAILURE;
		}
	}
}
