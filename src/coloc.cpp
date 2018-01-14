#include "stdafx.h"
#include "localizationData.hpp"
#include "localizationParams.hpp"
#include "featureDetector.hpp"
#include "featureMatcher.hpp"
#include "robustMatcher.hpp"
#include "mapBuilder.hpp"
#include "localizeImage.hpp"

#include <filesystem>

#if !defined _DEBUG
	#include "matplotlibcpp.h"
	namespace plt = matplotlibcpp;
#endif

using namespace coloc;
namespace fs = std::experimental::filesystem;

int main()
{
	coloc::LocalizationData data, data2;
	coloc::LocalizationParams params;

	params.featureDetectorType = "BINARY";
	params.imageSize = std::make_pair(640, 480);
	params.K << 320, 0, 320, 0, 320, 240, 0, 0, 1;
	params.imageFolder = "C:/Users/saihv/Desktop/testnew/";

	coloc::FeatureExtractor detector(params);
	coloc::FeatureMatcher matcher(params);
	coloc::RobustMatcher robustMatcher(params);
	coloc::Reconstructor reconstructor(params);
	coloc::Localizer localizer(params);
	coloc::Utils utils;

	unsigned int numDrones = 2;

	std::string filename[3];	

	filename[0] = "C:/Users/saihv/Desktop/testnew/img__Quad0_0000.png";
	filename[1] = "C:/Users/saihv/Desktop/testnew/img__Quad1_0000.png";
	filename[2] = "C:/Users/saihv/Desktop/testnew/img__Quad2_0000.png";

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
	bool mapReady = utils.setupMap(data, params);
	
	Pose3 pose;
	Cov6 cov;
	if (!mapReady)		
		localizer.localizeImage(filename[2], pose, data, cov);

	getchar();
	return 0;
}

