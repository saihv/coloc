#pragma once

#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/image_describer_akaze_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "nonFree/sift/SIFT_describer_io.hpp"

using namespace openMVG;

namespace coloc
{
	class FeatureExtractor {
	public:
		FeatureExtractor(std::string& method);
		std::unique_ptr<features::Regions> detectFeatures(std::string& imageName, std::string& method);
		// int drawFeaturePoints(std::string& imageName, features::PointFeatures points);

	private:
		std::unique_ptr<features::Image_describer> image_describer;
	};

	FeatureExtractor::FeatureExtractor(std::string& method)
	{
		if (method == "SIFT")
			image_describer.reset(new features::SIFT_Image_describer(features::SIFT_Image_describer::Params(), true));
		else if (method == "AKAZE")
			image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MLDB), true);
	}

	std::unique_ptr<openMVG::features::Regions> FeatureExtractor::detectFeatures(std::string& imageName, std::string& method)
	{
		image::Image<unsigned char> imageGray;

		if (!ReadImage(imageName.c_str(), &imageGray)) {
			std::cout << "Unable to read image from the given path." << std::endl;
		}
		auto regions = image_describer->Describe(imageGray, nullptr);

		return regions;
	}
}

