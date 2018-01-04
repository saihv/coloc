#include "stdafx.h"
#include "localizationData.hpp"
#include "featureDetector.hpp"
#include "featureMatcher.hpp"
#include "robustMatcher.hpp"
#include "mapBuilder.hpp"
#include "localizeImage.hpp"

int main()
{
	coloc::LocalizationData data;

	std::string detectorType = "AKAZE";
	std::pair <size_t, size_t> imageSize = std::make_pair(640, 480);
	Mat3 K;
	K << 320, 0, 320, 0, 320, 240, 0, 0, 1;

	coloc::FeatureExtractor detector(detectorType);
	coloc::FeatureMatcher matcher(detectorType);
	coloc::RobustMatcher robustMatcher(imageSize, K);
	coloc::Reconstructor reconstructor(imageSize, K);
	coloc::Localizer localizer(detectorType, imageSize, K);

	unsigned int numDrones = 3;

	std::string filename[3];
	std::string rootFolder = "C:/Users/saihv/Desktop/test/";
	filename[0] = "C:/Users/saihv/Desktop/test/img__Quad0_0.png";
	filename[1] = "C:/Users/saihv/Desktop/test/img__Quad1_0.png";
	filename[2] = "C:/Users/saihv/Desktop/test/img__Quad2_0.png";

	for (unsigned int i = 0; i < numDrones; ++i) {
		detector.detectFeatures(i, data.regions, filename[i]);
		detector.saveFeatureData(i, data.regions, filename[i]);

		data.scene.views[i].reset(new View(filename[i], i, 0, i, imageSize.first, imageSize.second));
	}

	matcher.computeMatches(data.regions, data.putativeMatches);
	robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches, data.relativePoses);
	reconstructor.reconstructScene(data);

	data.scene.s_root_path = rootFolder;
	localizer.setupMap(data);

	return 0;
}

