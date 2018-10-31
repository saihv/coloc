#pragma once

#include "stdafx.h"
#include "coloc/colocData.hpp"
#include "coloc/colocParams.hpp"
#include "coloc/CPUDetector.hpp"
#include "coloc/GPUDetector.hpp"
#include "coloc/CPUMatcher.hpp"
#include "coloc/GPUMatcher.hpp"
#include "coloc/FeatureDetector.hpp"
#include "coloc/FeatureMatcher.hpp"
#include "coloc/RobustMatcher.hpp"
#include "coloc/Reconstructor.hpp"
#include "coloc/Localizer.hpp"
#include "coloc/InterfaceDisk.hpp"
#include "coloc/InterfaceROS.hpp"
#include "coloc/logUtils.hpp"
#include "coloc/KalmanFilter.hpp"
#include "coloc/CovIntersection.hpp"
#include "coloc/rosUtils.hpp"

#include <chrono>
#include <ctime>

#include <ros/ros.h>

using namespace coloc;

class ColoC
{
public:
	ColoC(unsigned int& _nDrones, int& nImageStart, colocParams& _params, DetectorOptions& _dOpts, MatcherOptions& _mOpts)
		: params(_params), detector(_dOpts), matcher(_mOpts), robustMatcher(_params), reconstructor(_params),
		localizer(_params), colocInterface(_dOpts, _params, data), filter(_nDrones), rosUtils(_nDrones)
	{
		data.numDrones = _nDrones;
		for (unsigned int i = 0; i < data.numDrones; ++i) {
			data.filenames.push_back("");
			data.keyframeNames.push_back("");

			currentPoses.push_back(Pose3());
			currentCov.push_back(Cov6());
		}
		this->imageNumber = nImageStart;
		this->mapReady = false;
	}

	colocData data;
	colocParams params;
	DetectorOptions dOpts;
	MatcherOptions mOpts;
	int numDrones, imageNumber;

	//std::string poseFile;
	std::string mapFile;

	std::vector <std::string> filename;

private:
	//FeatureDetectorGPU gpuDetector{ 1.2f, 8, 640, 480, 5000 };
	//GPUMatcher gpuMatcher{ 5, 5000 };

	FeatureDetector <bool, CPUDetector> detector{ dOpts };
	FeatureMatcher <bool, CPUMatcher> matcher{ mOpts };
	RobustMatcher robustMatcher{ params };
	Reconstructor reconstructor{ params };
	Localizer localizer{ params };
	colocFilter filter{ data.numDrones };
	ROSUtils rosUtils { data.numDrones };
	CovIntersection covIntOptimizer;

	std::string matchesFile = params.imageFolder + "matches.svg";
	std::string poseFile = params.imageFolder + "poses.txt";
	std::string filtPoseFile = params.imageFolder + "poses_filtered.txt";

#ifdef USE_STREAM
	ROSInterface colocInterface;
#else
	DiskInterface colocInterface{ dOpts, params, data };
#endif
	Logger logger{};
	Utils utils;

	bool mapReady = false, stopThread = false, commandInter = false, updateMapNow = false;
	std::vector <Pose3> currentPoses;
	std::vector <Cov6> currentCov;

public:
	void mainThread()
	{
		std::vector <int> droneIds;
		std::vector <Pose3> poses;
		std::vector <Cov6> covs;

		std::string poseFile = params.imageFolder + "poses.txt";
		if (logger.createLogFile(poseFile) == EXIT_FAILURE)
			std::cout << "Cannot create log file for pose data";

		colocInterface.imageNumber = 0;
		ros::Rate loop_rate(20);

		for (int i = 0; i < data.numDrones; i++)
			droneIds.push_back(i);

		while (ros::ok()) {
			if (!mapReady) {
				colocInterface.processImages();
				initMap(droneIds, 1.0);
				mapReady = true;
				rosUtils.loadMapIntoPointCloudMsg(data);
			}

			while (colocInterface.imageNumber < 1) {
				Pose3 pose0, pose1, poseInter;
				Cov6 cov0 = std::vector<std::array<double, 36> >();
				Cov6 cov1 = std::vector<std::array<double, 36> >();

				int droneId = 0;
				colocInterface.processImageSingle(droneId);
				intraPoseEstimator(droneId, pose0, cov0);
				rosUtils.loadPoseIntoMsg(droneId, pose0);

				droneId = 1;
				colocInterface.processImageSingle(droneId);
				intraPoseEstimator(droneId, pose1, cov1);
				rosUtils.loadPoseIntoMsg(droneId, pose1);

				colocInterface.imageNumber++;
			}

			rosUtils.publishMsgs();
			loop_rate.sleep();
		}
	}

	void initMap(std::vector <int> droneIds, float scale = 1.0)
	{
		matcher.computeMatches(data.regions, data.putativeMatches);
		robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches, data.relativePoses);

		std::string matchesFile_putative = params.imageFolder + "matches_putative.svg";
		std::string matchesFile_geometric = params.imageFolder + "matches_geometric.svg";
		utils.drawMatches(matchesFile_putative, data.filenames[0], data.filenames[1], *data.regions[0].get(), *data.regions[1].get(), data.putativeMatches.at({ 0,1 }));
		utils.drawMatches(matchesFile_geometric, data.filenames[0], data.filenames[1], *data.regions[0].get(), *data.regions[1].get(), data.geometricMatches.at({ 0,1 }));

		Pose3 origin = Pose3(Mat3::Identity(), Vec3::Zero());

		std::cout << "Creating map" << std::endl;
		reconstructor.reconstructScene(0, data, origin, scale, true);

		std::string mapFile = params.imageFolder + "newmap.ply";
		logger.logMaptoPLY(data.scene, mapFile);
		data.scene.s_root_path = params.imageFolder;

		mapReady = data.setupMapDatabase(0);

		for (unsigned int i = 0; i < data.numDrones; ++i) 
			data.keyframeNames[i] = data.filenames[i];

		//gpuMatcher.setTrainingMap(data.mapRegions->RegionCount(), data.mapRegions->DescriptorRawData());
	}

	void intraPoseEstimator(int& droneId, Pose3& pose, Cov6& cov)
	{
		bool locStatus = false;
		float rmse = 10.0;
		int nTracks;
		IndMatches mapMatches;
		if (mapReady) {
			matcher.matchSceneWithMap(droneId, data, mapMatches);
			locStatus = localizer.localizeImage(droneId, pose, data, cov, rmse, mapMatches);
		}

		nTracks = mapMatches.size();
		std::cout << "Number of matches with map " << nTracks << std::endl;

		//std::string matchesFile = params.imageFolder + "matches.svg";
		//std::string poseFile = params.imageFolder + "poses.txt";
		//std::string filtPoseFile = params.imageFolder + "poses_filtered.txt";
		// std::string mapFileName = params.imageFolder + "img__Quad" + std::to_string(droneId) + "_0000.png";  //"image (" + number + ").png";
		// utils.drawMatches(matchesFile, fileName, mapFileName, *data.mapRegions.get(), *data.regions[0].get(), mapMatches);

		if (locStatus == EXIT_SUCCESS) {
			logger.logPoseCovtoFile(imageNumber, droneId, droneId, pose, cov, rmse, nTracks, poseFile);
			logger.logPosetoPLY(pose, mapFile);
		}
		else {
			if (cov.size() == 0) {
				double cov_pose[6 * 6] = { 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1 };
				std::array<double, 6 * 6> covpose;
				std::copy(std::begin(cov_pose), std::end(cov_pose), std::begin(covpose));
				cov.push_back(covpose);
			}

			Pose3 failurePose = Pose3(Mat3::Identity(), Vec3::Zero());
			logger.logPoseCovtoFile(imageNumber, droneId, droneId, failurePose, cov, rmse, nTracks, poseFile);
		}

		filter.fillMeasurements(filter.droneMeasurements[droneId], pose.center(), pose.rotation());
		filter.update(droneId, pose);
		logger.logPoseCovtoFile(imageNumber, droneId, droneId, pose, cov, rmse, nTracks, filtPoseFile);

		currentPoses[droneId] = pose;
		currentCov[droneId] = cov;
	}

	void interPoseEstimator(int sourceId, int destId, Pose3& origin, Pose3& pose, Cov6& cov)
	{
		std::string poseFile = params.imageFolder + "poses.txt";
		std::string matchesFile = "matches" + std::to_string(colocInterface.imageNumber) + ".svg";

		// colocData tempScene;

		//for (auto &region : data.regions)
		//	tempScene.regions.emplace_hint(tempScene.regions.end(), region.first, region.second.get());

		for (unsigned int i = 0; i < 2; ++i) {
			data.tempScene.views[i].reset(new View(data.filenames[i], i, 0, i, params.imageSize.first, params.imageSize.second));
		}

		Pair interPosePair = std::make_pair <IndexT, IndexT>((IndexT)sourceId, (IndexT)destId);

		IndMatches pairMatches;
		matcher.computeMatchesPair(interPosePair, data.regions, pairMatches);

		data.putativeMatches.insert({ interPosePair, std::move(pairMatches) });
		robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches, data.relativePoses);

		// utils.drawMatches(matchesFile, data.filenames[0], data.filenames[1], *data.regions[0].get(), *data.regions[1].get(), data.geometricMatches.at({ 0,1 }));

		std::cout << "Creating temporary map" << std::endl;
		coloc::Reconstructor tempReconstructor(params);
		coloc::Utils tempUtils;
		tempReconstructor.reconstructScene(true, data, Pose3(), 1.0, false);

		PoseRefiner refiner;
		float rmse;
		const Optimize_Options refinementOptions(Intrinsic_Parameter_Type::NONE, Extrinsic_Parameter_Type::ADJUST_ALL, Structure_Parameter_Type::ADJUST_ALL);
		bool locStatus = refiner.refinePose(data.tempScene, refinementOptions, rmse, cov);

		data.tempScene.s_root_path = params.imageFolder;

		bool isInter = true;
		mapReady = data.setupMapDatabase(isInter);

		std::vector <IndMatch> commonFeatures;
		robustMatcher.matchMaps(data.mapRegions, data.interMapRegions, commonFeatures);
		std::string matchesFileMap = params.imageFolder + "matchesMap_" + std::to_string(destId) + "_" + std::to_string(imageNumber) + ".svg";

		utils.drawMatches(matchesFileMap, data.keyframeNames[0], data.filenames[0], *data.mapRegions.get(), *data.interMapRegions.get(), commonFeatures);
		double scaleDiff = utils.computeScaleDifference(data.scene, data.mapRegionIdx, data.tempScene, data.interMapRegionIdx, commonFeatures);

		std::cout << "Found scale difference to be " << scaleDiff << std::endl;
		utils.rescaleMap(data.tempScene, scaleDiff);

		const Optimize_Options refinementOptions2(Intrinsic_Parameter_Type::NONE, Extrinsic_Parameter_Type::ADJUST_ALL, Structure_Parameter_Type::NONE);
		locStatus = refiner.refinePose(data.tempScene, refinementOptions2, rmse, cov);

		if (cov.size() == 0) {
			double cov_pose[6 * 6] = { 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1 };
			std::array<double, 6 * 6> covpose;
			std::copy(std::begin(cov_pose), std::end(cov_pose), std::begin(covpose));
			cov.push_back(covpose);
		}

		pose = data.tempScene.poses.at(1);
		std::string newMapFile = params.imageFolder + "newmap.ply";
		int tracks = 0;

		if (locStatus) {
			logger.logPoseCovtoFile(imageNumber, destId, sourceId, pose, cov, rmse, tracks, poseFile);
			//logger.logPosetoPLY(pose, newMapFile);
			//logger.logMaptoPLY(tempScene.scene, newMapFile);
		}

		matrix <double, 3, 3> Cs, Cd;
		matrix <double, 3, 1> xs, xd;

		utils.loadPoseCovariance(currentCov[destId], Cs);
		utils.loadPoseCovariance(cov, Cd);
		
		xd = pose.center()[0], pose.center()[1], pose.center()[2];
		xs = currentPoses[destId].center()[0], currentPoses[destId].center()[1], currentPoses[destId].center()[2];

		covIntOptimizer.loadData(Cs, Cd, xs, xd);
		covIntOptimizer.optimize();

		Pose3 poseFused = pose;
		Cov6 covFused = cov;

		utils.readPoseCovariance(Cs, covFused);

		pose.center()[0] = covIntOptimizer.poseFused(0);
		pose.center()[1] = covIntOptimizer.poseFused(1);
		pose.center()[2] = covIntOptimizer.poseFused(2);

		logger.logPoseCovtoFile(imageNumber, destId, sourceId, pose, cov, rmse, tracks, filtPoseFile);
	}

	void updateMap(std::vector <int> drones)
	{
		colocData updateData;

		//std::string filename;
		std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);
		for (unsigned int i = 0; i < drones.size(); ++i) {
			data.filenames[i] = params.imageFolder + "img__Quad" + std::to_string(drones[i]) + "_" + number + ".png";
			std::cout << data.filenames[i] << std::endl;

			//detector.detectFeatures(i, updateData.regions, filename);
			//detector.saveFeatureData(i, updateData.regions, filename);

			updateData.scene.views[i].reset(new View(data.filenames[i], i, 0, i, params.imageSize.first, params.imageSize.second));
		}

		matcher.computeMatches(updateData.regions, updateData.putativeMatches);
		robustMatcher.filterMatches(updateData.regions, updateData.putativeMatches, updateData.geometricMatches, updateData.relativePoses);

		Pose3 origin = Pose3(Mat3::Identity(), Vec3::Zero());

		std::cout << "Updating map" << std::endl;
		Reconstructor updateReconstructor(params);
		updateReconstructor.reconstructScene(0, updateData, origin, 1.0, true);

		bool newMapReady = updateData.setupMapDatabase(0);

		if (newMapReady) {
			std::vector <IndMatch> commonFeatures;
			robustMatcher.matchMaps(data.mapRegions, updateData.mapRegions, commonFeatures);
			double scaleDiff = utils.computeScaleDifference(data.scene, data.mapRegionIdx, updateData.scene, updateData.mapRegionIdx, commonFeatures);
			std::cout << "Scale factor ratio computed during update as " << scaleDiff << std::endl;
			utils.rescaleMap(updateData.scene, scaleDiff);
		}

		std::string newMapFile = params.imageFolder + "newmap.ply";
		logger.logMaptoPLY(updateData.scene, newMapFile);

		data = updateData;
		data.setupMapDatabase(0);

		//logger.logMaptoPLY(data.scene, mapFile);
		//data.scene.s_root_path = params.imageFolder;
		//mapReady = utils.setupMap(data, params);
	}
};