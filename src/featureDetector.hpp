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
		FeatureExtractor(LocalizationParams& params);
		bool detectFeatures(unsigned int index, FeatureMap& regions, std::string& imageName);
		bool saveFeatureData(uint16_t id, FeatureMap& regions, std::string& name);	

	private:
		std::unique_ptr<features::Image_describer> image_describer;
	};

	FeatureExtractor::FeatureExtractor(LocalizationParams& params)
	{
		if (params.featureDetectorType == "SIFT")
			image_describer.reset(new features::SIFT_Image_describer(features::SIFT_Image_describer::Params(), true));
		else if (params.featureDetectorType == "AKAZE")
			image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MSURF), true);
		else if (params.featureDetectorType == "BINARY")
			image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MLDB), true);
	}

	bool FeatureExtractor::detectFeatures(unsigned int index, FeatureMap &regions, std::string &imageName)
	{
		image::Image<unsigned char> imageGray;

		if (!ReadImage(imageName.c_str(), &imageGray)) {
			std::cout << "Unable to read image from the given path." << std::endl;
		}
		if (!image_describer->Describe(imageGray, regions[index])) {
			std::cout << "Feature detection failed.";
			return Failure;
		}
		return Success;
	}

	bool FeatureExtractor::saveFeatureData(uint16_t id, FeatureMap &regions, std::string &name)
	{
		const std::string
			sFeat = stlplus::create_filespec(stlplus::folder_part(name), stlplus::basename_part(name), "feat"),
			sDesc = stlplus::create_filespec(stlplus::folder_part(name), stlplus::basename_part(name), "desc");

		if (!image_describer->Save(regions.at(id).get(), sFeat, sDesc))
			return Failure;
		else
			return Success;
	}
}

