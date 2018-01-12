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
	coloc::LocalizationData data, data2;
	coloc::LocalizationParams params;

	params.featureDetectorType = "AKAZE";
	params.imageSize = std::make_pair(640, 480);
	params.K << 320, 0, 320, 0, 320, 240, 0, 0, 1;
	params.imageFolder = "C:/Users/saihv/Desktop/testnewer/";

	coloc::FeatureExtractor detector(params);
	coloc::FeatureMatcher matcher(params);
	coloc::RobustMatcher robustMatcher(params);
	coloc::Reconstructor reconstructor(params);
	coloc::Localizer localizer(params);
	coloc::Utils utils;

	unsigned int numDrones = 2;

	std::string filename[3];	

	filename[0] = "C:/Users/saihv/Desktop/testnewer/img__Quad0_0000.png";
	filename[1] = "C:/Users/saihv/Desktop/testnewer/img__Quad1_0002.png";
	filename[2] = "C:/Users/saihv/Desktop/testnewer/img__Quad1_0250.png";

	for (unsigned int i = 0; i < numDrones; ++i) {
		detector.detectFeatures(i, data.regions, filename[i]);
		detector.saveFeatureData(i, data.regions, filename[i]);

		data.scene.views[i].reset(new View(filename[i], i, 0, i, params.imageSize.first, params.imageSize.second));
	}

	matcher.computeMatches(data.regions, data.putativeMatches);
	robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches, data.relativePoses);

	Pose3 origin = Pose3(Mat3::Identity(), Vec3::Zero());

	reconstructor.reconstructScene(data, origin, 1.0);

	data.scene.s_root_path = params.imageFolder;
	bool mapReady = localizer.setupMap(data);
	
	Pose3 pose;
	if (!mapReady) {
		
		localizer.localizeImage(filename[2], pose, data);
	}
	
	for (unsigned int i = 0; i < numDrones; ++i) {
		detector.detectFeatures(i, data2.regions, filename[i]);
		detector.saveFeatureData(i, data2.regions, filename[i]);

		data2.scene.views[i].reset(new View(filename[i], i, 0, i, params.imageSize.first, params.imageSize.second));
	}

	matcher.computeMatches(data2.regions, data2.putativeMatches);
	robustMatcher.filterMatches(data2.regions, data2.putativeMatches, data2.geometricMatches, data2.relativePoses);

	coloc::Reconstructor reconstructor2(params);
	reconstructor2.reconstructScene(data2, origin, 1.0);

	coloc::Utils utils2;

	data2.scene.s_root_path = params.imageFolder;
	mapReady = utils2.setupMap(data2, params);

	float scaleDiff;
	utils.computeScaleDifference(data, data2, scaleDiff);

	std::cout << "Difference between scale factors is " << scaleDiff << std::endl;

	

	return 0;
}

