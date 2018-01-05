#include "stdafx.h"
#include "localizationData.hpp"
#include "localizationParams.hpp"
#include "featureDetector.hpp"
#include "featureMatcher.hpp"
#include "robustMatcher.hpp"
#include "mapBuilder.hpp"
#include "localizeImage.hpp"

#include <filesystem>
namespace fs = std::experimental::filesystem;

int main()
{
	coloc::LocalizationData data;
	coloc::LocalizationParams params;

	params.featureDetectorType = "AKAZE";
	params.imageSize = std::make_pair(640, 480);
	params.K << 320, 0, 320, 0, 320, 240, 0, 0, 1;
	params.imageFolder = "C:/Users/saihv/Desktop/test/";

	coloc::FeatureExtractor detector(params);
	coloc::FeatureMatcher matcher(params);
	coloc::RobustMatcher robustMatcher(params);
	coloc::Reconstructor reconstructor(params);
	coloc::Localizer localizer(params);

	unsigned int numDrones = 3;

	std::string filename[3];	

	filename[0] = "C:/Users/saihv/Desktop/test/img__Quad0_0.png";
	filename[1] = "C:/Users/saihv/Desktop/test/img__Quad1_0.png";
	filename[2] = "C:/Users/saihv/Desktop/test/img__Quad2_0.png";

	for (unsigned int i = 0; i < numDrones; ++i) {
		detector.detectFeatures(i, data.regions, filename[i]);
		detector.saveFeatureData(i, data.regions, filename[i]);

		data.scene.views[i].reset(new View(filename[i], i, 0, i, params.imageSize.first, params.imageSize.second));
	}

	matcher.computeMatches(data.regions, data.putativeMatches);
	robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches, data.relativePoses);
	reconstructor.reconstructScene(data);

	data.scene.s_root_path = params.imageFolder;
	bool mapReady = localizer.setupMap(data);

	if (!mapReady) {
		Pose3 pose;
		localizer.localizeImage(filename[1], pose);
	}

	return 0;
}

