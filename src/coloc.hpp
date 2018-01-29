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
#include "logUtils.hpp"
#include <filesystem>
#include <chrono>
#include <ctime>

namespace coloc
{
	class ColoC
	{
	public:
		ColoC(int& nDrones, int& nImageStart, LocalizationParams& params)
			: params(params), detector(params), matcher(params), robustMatcher(params), reconstructor(params),
			localizer(params)
		{
			this->numDrones = nDrones;
			this->imageNumber = nImageStart;
			this->mapReady = false;
		}
		void intraPoseEstimator(int& droneId, Pose3& pose, Cov6& cov);
		void interPoseEstimator(int sourceId, int destId, Pose3& origin, Pose3& pose, Cov6& cov);
		void initMap(std::vector <int> droneIds, float scale);
		void updateMap(std::vector <int> drones);

		LocalizationData data;
		LocalizationParams params;
		int numDrones, imageNumber;

		std::string poseFile;
		std::string mapFile;

	private:
		FeatureExtractor detector{ params };
		FeatureMatcher matcher{ params };
		RobustMatcher robustMatcher{ params };
		Reconstructor reconstructor{ params };
		Localizer localizer{ params };
		Logger logger{};
		Utils utils;
		bool mapReady;
	};

	void ColoC::initMap(std::vector <int> droneIds, float scale = 1.0)
	{
		std::vector <std::string> filename;
		std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);
		auto start = std::chrono::system_clock::now();
		for (unsigned int i = 0; i < droneIds.size(); ++i) {
			filename.push_back(params.imageFolder + "img__Quad" + std::to_string(droneIds[i]) + "_" + number + ".png");
			std::cout << filename[i] << std::endl;

			detector.detectFeatures(i, data.regions, filename[i]);
			detector.saveFeatureData(i, data.regions, filename[i]);

			data.scene.views[i].reset(new View(filename[i], i, 0, i, params.imageSize.first, params.imageSize.second));
		}
		auto end = std::chrono::system_clock::now();

		std::chrono::duration<double> elapsed_seconds = end - start;
		std::time_t end_time = std::chrono::system_clock::to_time_t(end);
		std::cout << "Feature detection took "  << elapsed_seconds.count() << "s\n";

		start = std::chrono::system_clock::now();
		matcher.computeMatches(data.regions, data.putativeMatches);
		robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches, data.relativePoses);
		end = std::chrono::system_clock::now();

		elapsed_seconds = end - start;
		end_time = std::chrono::system_clock::to_time_t(end);
		std::cout << "Feature matching took " << elapsed_seconds.count() << "s\n";
		std::string matchesFile = "matches.svg";
		utils.drawMatches(matchesFile, filename[0], filename[1], *data.regions[0].get(), *data.regions[1].get(), data.geometricMatches.at({ 0,1 }));
		Pose3 origin = Pose3(Mat3::Identity(), Vec3::Zero());
		start = std::chrono::system_clock::now();
		std::cout << "Updating map" << std::endl;
		reconstructor.reconstructScene(data, origin, scale, true);
		end = std::chrono::system_clock::now();
		elapsed_seconds = end - start;
		end_time = std::chrono::system_clock::to_time_t(end);
		std::cout << "Reconstruction took " << elapsed_seconds.count() << "s\n";
		logger.logMaptoPLY(data.scene, mapFile);
		data.scene.s_root_path = params.imageFolder;
		start = std::chrono::system_clock::now();
		mapReady = data.setupFeatureDatabase(params);
		end = std::chrono::system_clock::now();
		elapsed_seconds = end - start;
		end_time = std::chrono::system_clock::to_time_t(end);
		std::cout << "Database setup took " << elapsed_seconds.count() << "s\n";
	}

	void ColoC::intraPoseEstimator(int& droneId, Pose3& pose, Cov6& cov)
	{
		auto start = std::chrono::system_clock::now();
		std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);
		std::string fileName = params.imageFolder + "img__Quad" + std::to_string(droneId) + "_" + number + ".png";
		bool locStatus = false;
		float rmse;

		if (mapReady)
			locStatus = localizer.localizeImage(fileName, pose, data, cov, rmse);

		if (locStatus) {
			logger.logPoseCovtoFile(imageNumber, droneId, droneId, pose, cov, rmse, poseFile);
			logger.logPosetoPLY(pose, mapFile);
		}

		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::time_t end_time = std::chrono::system_clock::to_time_t(end);
		std::cout << "Intra estimation took " << elapsed_seconds.count() << "s\n";
	}

	void ColoC::interPoseEstimator(int sourceId, int destId, Pose3& origin, Pose3& pose, Cov6& cov)
	{
		auto start = std::chrono::system_clock::now();
		std::vector <std::string> filename;
		std::string matchesFile = "matches.svg";
		std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);

		filename.push_back(params.imageFolder + "img__Quad" + std::to_string(sourceId) + "_" + number + ".png");
		filename.push_back(params.imageFolder + "img__Quad" + std::to_string(destId) + "_" + number + ".png");

		LocalizationData tempScene;

		for (unsigned int i = 0; i < 2; ++i) {
			detector.detectFeatures(i, tempScene.regions, filename[i]);
			detector.saveFeatureData(i, tempScene.regions, filename[i]);

			tempScene.scene.views[i].reset(new View(filename[i], i, 0, i, params.imageSize.first, params.imageSize.second));
		}

		matcher.computeMatches(tempScene.regions, tempScene.putativeMatches);
		robustMatcher.filterMatches(tempScene.regions, tempScene.putativeMatches, tempScene.geometricMatches, tempScene.relativePoses);

		utils.drawMatches(matchesFile, filename[0], filename[1], *tempScene.regions[0].get(), *tempScene.regions[1].get(), tempScene.geometricMatches.at({ 0,1 }));

		std::cout << "Creating temporary map" << std::endl;
		coloc::Reconstructor tempReconstructor(params);
		coloc::Utils tempUtils;
		tempReconstructor.reconstructScene(tempScene, origin, 1.0, false);

		PoseRefiner refiner;
		float rmse;
		const Optimize_Options refinementOptions(Intrinsic_Parameter_Type::NONE, Extrinsic_Parameter_Type::ADJUST_ALL, Structure_Parameter_Type::NONE);
		bool locStatus = refiner.refinePose(tempScene.scene, refinementOptions, rmse, cov);

		tempScene.scene.s_root_path = params.imageFolder;
		mapReady = tempScene.setupFeatureDatabase(params);

		std::vector <IndMatch> commonFeatures;
		robustMatcher.matchMaps(data, tempScene, commonFeatures);

		utils.drawMatches(matchesFile, filename[0], filename[0], *data.mapRegions.get(), *tempScene.mapRegions.get(), commonFeatures);
		double scaleDiff = utils.computeScaleDifference(params, data, tempScene, commonFeatures);

		std::cout << "Found scale difference to be " << scaleDiff << std::endl;
		utils.rescaleMap(tempScene.scene, scaleDiff);

		pose = tempScene.scene.poses.at(1);
		std::string newMapFile = params.imageFolder + "newmap.ply";

		if (locStatus) {
			logger.logPoseCovtoFile(imageNumber, destId, sourceId, pose, cov, rmse, poseFile);
			//logger.logPosetoPLY(pose, newMapFile);
			//logger.logMaptoPLY(tempScene.scene, newMapFile);
		}

		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::time_t end_time = std::chrono::system_clock::to_time_t(end);
		std::cout << "Inter estimation took " << elapsed_seconds.count() << "s\n";
	}

	void ColoC::updateMap(std::vector <int> drones)
	{
		LocalizationData updateData;

		std::string filename;
		std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);
		for (unsigned int i = 0; i < drones.size(); ++i) {
			filename = params.imageFolder + "img__Quad" + std::to_string(drones[i]) + "_" + number + ".png";
			std::cout << filename << std::endl;

			detector.detectFeatures(i, updateData.regions, filename);
			detector.saveFeatureData(i, updateData.regions, filename);

			updateData.scene.views[i].reset(new View(filename, i, 0, i, params.imageSize.first, params.imageSize.second));
		}

		matcher.computeMatches(updateData.regions, updateData.putativeMatches);
		robustMatcher.filterMatches(updateData.regions, updateData.putativeMatches, updateData.geometricMatches, updateData.relativePoses);

		Pose3 origin = Pose3(Mat3::Identity(), Vec3::Zero());

		std::cout << "Updating map" << std::endl;
		Reconstructor updateReconstructor(params);
		updateReconstructor.reconstructScene(updateData, origin, 1.0, true);

		bool newMapReady = updateData.setupFeatureDatabase(params);

		if (newMapReady) {
			std::vector <IndMatch> commonFeatures;
			robustMatcher.matchMaps(data, updateData, commonFeatures);
			double scaleDiff = utils.computeScaleDifference(params, data, updateData, commonFeatures);
			std::cout << "Scale factor ratio computed during update as " << scaleDiff << std::endl;
			utils.rescaleMap(updateData.scene, scaleDiff);
		}

		std::string newMapFile = params.imageFolder + "newmap.ply";
		logger.logMaptoPLY(updateData.scene, newMapFile);

		data = updateData;
		data.setupFeatureDatabase(params);

		//logger.logMaptoPLY(data.scene, mapFile);
		//data.scene.s_root_path = params.imageFolder;
		//mapReady = utils.setupMap(data, params);
	}
}
