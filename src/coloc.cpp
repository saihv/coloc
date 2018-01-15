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
		void interPoseEstimator(int sourceId, int destId, Pose3& origin, Pose3& pose, Cov6& cov);
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
		reconstructor.reconstructScene(data, origin, scale, true);

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

	void ColoC::interPoseEstimator(int sourceId, int destId, Pose3& origin, Pose3& pose, Cov6& cov)
	{
		std::vector <std::string> filename;
		std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);

		filename.push_back(params.imageFolder + "img__Quad" + std::to_string(sourceId) + "_" + number + ".png");
		filename.push_back(params.imageFolder + "img__Quad" + std::to_string(destId) + "_" + number + ".png");

		LocalizationData tempScene;

		for (unsigned int i = 0; i < numDrones; ++i) {
			detector.detectFeatures(i, tempScene.regions, filename[i]);
			detector.saveFeatureData(i, tempScene.regions, filename[i]);

			tempScene.scene.views[i].reset(new View(filename[i], i, 0, i, params.imageSize.first, params.imageSize.second));
		}

		matcher.computeMatches(tempScene.regions, tempScene.putativeMatches);
		robustMatcher.filterMatches(tempScene.regions, tempScene.putativeMatches, tempScene.geometricMatches, tempScene.relativePoses);

		std::cout << "Updating map" << std::endl;
		coloc::Reconstructor tempReconstructor(params);
		coloc::Utils tempUtils;
		tempReconstructor.reconstructScene(tempScene, origin, 1.0, false);

		tempScene.scene.s_root_path = params.imageFolder;
		mapReady = tempUtils.setupMap(tempScene, params);

		float scaleDiff;
		utils.computeScaleDifference(data, tempScene, scaleDiff);
		utils.rescaleMap(tempScene.scene, scaleDiff);

		PoseRefiner refiner;
		const Optimize_Options refinementOptions(Intrinsic_Parameter_Type::NONE, Extrinsic_Parameter_Type::ADJUST_ALL, Structure_Parameter_Type::NONE);
		refiner.refinePose(tempScene.scene, refinementOptions, cov);
		pose = tempScene.scene.poses.at(1);
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

	Pose3 poseIntra, poseInter;
	Cov6 covIntra, covInter;

	int droneId = 2;
	coloc.intraPoseEstimator(droneId, poseIntra, covIntra);
	coloc.interPoseEstimator(0, 2, coloc.data.scene.poses.at(0), poseInter, covInter);
	Plotter plotter;
	plotter.plotScene(coloc.data.scene);
	plotter.plotPoseandCovariance(poseIntra, covIntra);
	plotter.plotPoseandCovariance(poseInter, covInter);
	plotter.drawPlot();
	getchar();
	return 0;
}

