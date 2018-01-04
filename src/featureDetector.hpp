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
		void detectFeatures(unsigned int index, std::map<IndexT, std::unique_ptr<features::Regions> >& regions, std::string& imageName);
		void saveFeatureData(uint16_t id, std::map<IndexT, std::unique_ptr<features::Regions> >& regions, std::string& name);
		// int drawFeaturePoints(std::string& imageName, features::PointFeatures points);

	private:
		std::unique_ptr<features::Image_describer> image_describer;
	};

	FeatureExtractor::FeatureExtractor(LocalizationParams& params)
	{
		if (params.featureDetectorType == "SIFT")
			image_describer.reset(new features::SIFT_Image_describer(features::SIFT_Image_describer::Params(), true));
		else if (params.featureDetectorType == "AKAZE")
			image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MLDB), true);
	}

	void FeatureExtractor::detectFeatures(unsigned int index, std::map<IndexT, std::unique_ptr<features::Regions> >& regions, std::string& imageName)
	{
		image::Image<unsigned char> imageGray;

		if (!ReadImage(imageName.c_str(), &imageGray)) {
			std::cout << "Unable to read image from the given path." << std::endl;
		}
		image_describer->Describe(imageGray, regions[index]);
	}

	void FeatureExtractor::saveFeatureData(uint16_t id, std::map<IndexT, std::unique_ptr<features::Regions> >& regions, std::string& name)
	{
		const std::string
			sFeat = stlplus::create_filespec(stlplus::folder_part(name), stlplus::basename_part(name), "feat"),
			sDesc = stlplus::create_filespec(stlplus::folder_part(name), stlplus::basename_part(name), "desc");

		image_describer->Save(regions.at(id).get(), sFeat, sDesc);
	}
}

