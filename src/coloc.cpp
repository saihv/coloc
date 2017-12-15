#include "stdafx.h"
#include "localizationData.hpp"
#include "featureDetector.hpp"

int main()
{
	coloc::LocalizationData data;
	std::string detectorType = "AKAZE";
	coloc::FeatureExtractor detector(detectorType);


	std::string filename;
	detector.detectFeatures(data.regions, filename);
}

