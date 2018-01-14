#pragma once

#include "stdafx.h"
#include "localizationData.hpp"
#include "localizationParams.hpp"
#include "featureDetector.hpp"
#include "featureMatcher.hpp"
#include "robustMatcher.hpp"
#include "mapBuilder.hpp"
#include "localizeImage.hpp"
#include "plotUtils.hpp"

#include <filesystem>

namespace coloc
{
	class ColoC
	{
	public:
		ColoC(int& nDrones, int& nImageStart, LocalizationParams& params) 
			: detector(params), matcher(params), robustMatcher(params), reconstructor(params),
			  localizer(params)
		{
			this->numDrones = nDrones;
			this->imageNumber = nImageStart; 
			this->params = params;
		}
		void intraPoseEstimator(int& droneId, Pose3& pose, Cov6& cov);
		void interPoseEstimator();
		void updateMap(float scale);

		LocalizationData data;
		LocalizationParams params;
		int numDrones, imageNumber;

	private:
		FeatureExtractor detector{ params };
		FeatureMatcher matcher{ params };
		RobustMatcher robustMatcher{ params };
		Reconstructor reconstructor{ params };
		Localizer localizer{ params };
		Utils utils;
		bool mapReady;
	};

	void ColoC::updateMap(float scale = 1.0)
	{
		std::string filename;
		std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);
		for (unsigned int i = 0; i < numDrones; ++i) {
			filename = params.imageFolder + "img__Quad" + std::to_string(i) + "_" + number + ".png";
			std::cout << filename << std::endl;

			detector.detectFeatures(i, data.regions, filename);
			detector.saveFeatureData(i, data.regions, filename);

			data.scene.views[i].reset(new View(filename, i, 0, i, params.imageSize.first, params.imageSize.second));
		}

		matcher.computeMatches(data.regions, data.putativeMatches);
		robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches, data.relativePoses);

		Pose3 origin = Pose3(Mat3::Identity(), Vec3::Zero());

		std::cout << "Updating map" << std::endl;
		reconstructor.reconstructScene(data, origin, scale);

		data.scene.s_root_path = params.imageFolder;
		mapReady = utils.setupMap(data, params);
	}

	void ColoC::intraPoseEstimator(int& droneId, Pose3& pose, Cov6& cov)
	{
		std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);
		std::string fileName = params.imageFolder + "img__Quad" + std::to_string(2) + "_" + number + ".png";
		if (!mapReady)
			localizer.localizeImage(fileName, pose, data, cov);
	}
}

using namespace coloc;
namespace fs = std::experimental::filesystem;

LocalizationParams initParams()
{
	LocalizationParams params;
	params.featureDetectorType = "BINARY";
	params.imageSize = std::make_pair(640, 480);
	params.K << 320, 0, 320, 0, 320, 240, 0, 0, 1;
	params.imageFolder = "C:/Users/saihv/Desktop/testnew/";
	return params;
}

int main()
{
	int numDrones = 2;
	int startNum = 0;
	LocalizationParams params = initParams();

	ColoC coloc(numDrones, startNum, params);
	coloc.updateMap(10.0);

	Pose3 pose;
	Cov6 cov;

	int droneId = 2;
	coloc.intraPoseEstimator(droneId, pose, cov);

	Plotter plotter;
	plotter.plotScene(coloc.data.scene);
	plotter.plotPoseandCovariance(pose, cov);
	plotter.drawPlot();
	getchar();
	return 0;
}

