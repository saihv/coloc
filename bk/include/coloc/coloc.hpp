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

#include <filesystem>
#include <chrono>
#include <ctime>

#define DEBUG 1

using namespace coloc;

class ColoC
{
public:
	ColoC(unsigned int& _nDrones, int& nImageStart, colocParams& _params, DetectorOptions& _dOpts, MatcherOptions& _mOpts)
		: params(_params), detector(_dOpts), matcher(_mOpts), robustMatcher(_params), reconstructor(_params),
		localizer(_params), colocInterface(_dOpts, _params, data), filter(_nDrones)
	{
		data.numDrones = _nDrones;
		for (unsigned int i = 0; i < data.numDrones; ++i) {
			data.filenames.push_back("");
			data.keyframeNames.push_back("");

			currentPoses.push_back(Pose3(Mat3::Identity(), Vec3::Zero()));
			currentCov.push_back(Cov6());
			trackCounts.push_back(0);
		}
		this->imageNumber = nImageStart;
		this->mapReady = false;
	}
	colocData data;
	colocParams params;
	DetectorOptions dOpts;
	MatcherOptions mOpts;
	int numDrones, imageNumber;
	int updateNum = 0;

	//std::string poseFile;
	std::string mapFile;

	std::vector <std::string> filename;

private:
	//FeatureDetectorGPU gpuDetector{ 1.2f, 8, 640, 480, 5000 };
	//GPUMatcher gpuMatcher{ 5, 5000 };

#ifdef USE_CUDA
	FeatureDetector <bool, GPUDetector> detector{ dOpts };
	FeatureMatcher <bool, GPUMatcher> matcher{ mOpts };
#else
	FeatureDetector <bool, CPUDetector> detector{ dOpts };
	FeatureMatcher <bool, CPUMatcher> matcher{ mOpts };
#endif
	RobustMatcher robustMatcher{ params };
	Reconstructor reconstructor{ params };
	Localizer localizer{ params };
	colocFilter filter{ data.numDrones };
	CovIntersection covIntOptimizer;

	std::string matchesFile = params.imageFolder + "matches.svg";
	std::string poseFile = params.imageFolder + "poses.txt";
	std::string filtPoseFile = params.imageFolder + "poses_filtered.txt";

	std::string seedMapFile;

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
	std::vector <int> trackCounts;

public:
	void mainThread()
	{
		std::vector <int> droneIds;
		std::vector <Pose3> poses;
		std::vector <Cov6> covs;

		if (logger.createLogFile(poseFile) == EXIT_FAILURE || logger.createLogFile(filtPoseFile) == EXIT_FAILURE)
			std::cout << "Cannot create log file for pose data";

		colocInterface.imageNumber = 0;

		for (int i = 0; i < data.numDrones; i++)
			droneIds.push_back(i);

		int ctr = 0;
		while (!stopThread) {
			if (!mapReady) {
				colocInterface.processImages(droneIds);
				initMap(droneIds, 3.0);
				mapReady = true;
				colocInterface.imageNumber = 1;
			}
			
			this->imageNumber = colocInterface.imageNumber;
			
			//while (colocInterface.imageNumber < 800) {
				for (int i = 0; i < 2; ++i) {
					colocInterface.processImageSingle(droneIds[i]);
					intraPoseEstimator(i, currentPoses[i], currentCov[i]);
				}
			//	colocInterface.imageNumber++;
			//}

				interPoseEstimator(0, 1);
			//if (colocInterface.imageNumber == 800)
				stopThread = true;
		}
	}

	void initMap(std::vector <int> droneIds, float scale = 1.0)
	{
		matcher.computeMatches(data.regions, data.putativeMatches);
		robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches, data.relativePoses);

#ifdef DEBUG
		std::string matchesFile_putative = params.imageFolder + "matches_putative.svg";
		std::string matchesFile_geometric = params.imageFolder + "matches_geometric.svg";
		utils.drawMatches(params.imageSize, matchesFile_putative, data.filenames[0], data.filenames[1], *data.regions[0].get(), *data.regions[1].get(), data.putativeMatches.at({ 0,1 }));
		utils.drawMatches(params.imageSize, matchesFile_geometric, data.filenames[0], data.filenames[1], *data.regions[0].get(), *data.regions[1].get(), data.geometricMatches.at({ 0,1 }));
#endif

		std::cout << "Creating map" << std::endl;
		reconstructor.reconstructScene(0, data, currentPoses, scale, true);

		std::string mapFile = params.imageFolder + "newmap.ply";
		logger.logMaptoPLY(data.scene, mapFile);
		data.scene.s_root_path = params.imageFolder;

		mapReady = data.setupMapDatabase(0);
#ifdef DEBUG
		std::string mapFeatFile = params.imageFolder + "OriginalMap_Features.svg";
		utils.drawFeatures(data.filenames[0], params.imageSize, data.mapRegions->Features(), mapFeatFile);
#endif
		for (unsigned int i = 0; i < data.numDrones; ++i) 
			data.keyframeNames[i] = data.filenames[i];

#ifdef USE_CUDA
		matcher.setMapData(data.mapRegions->RegionCount(), const_cast<unsigned int*>(static_cast<const unsigned int*>(data.mapRegions->DescriptorRawData())));
#endif
	}

	void intraPoseEstimator(int& droneId, Pose3& pose, Cov6& cov)
	{
		std::cout << colocInterface.imageNumber << std::endl;

		bool locStatus = false;
		float rmse = 10.0;
		int nTracks;
		IndMatches mapMatches, inlierMatches;
		std::vector <uint32_t> inliers;
		if (mapReady) {
			matcher.matchSceneWithMap(droneId, data, mapMatches);
			locStatus = localizer.localizeImage(droneId, pose, data, cov, rmse, mapMatches, inliers);
		}
		
		nTracks = inliers.size();
		std::cout << "Number of matches with map " << nTracks << std::endl;
		trackCounts[droneId] = nTracks;

#ifdef DEBUG
		for (int i = 0; i < inliers.size(); i++)
			inlierMatches.push_back(mapMatches[i]);
		std::string matchesFile = params.imageFolder + "matchesIMG" + std::to_string(colocInterface.imageNumber) + ".svg";
		std::string number = std::string(4 - std::to_string(colocInterface.imageNumber).length(), '0') + std::to_string(colocInterface.imageNumber);
		std::string filename = params.imageFolder + "img__Quad" + std::to_string(droneId) + "_" + number + ".png";
		utils.drawMatches(params.imageSize, matchesFile, data.keyframeNames[0], filename, *data.mapRegions.get(), *data.regions[droneId].get(), inlierMatches);
#endif

		if (locStatus == EXIT_SUCCESS) {
			logger.logPoseCovtoFile(colocInterface.imageNumber, droneId, droneId, pose, cov, rmse, nTracks, poseFile);
			logger.logPosetoPLY(pose, mapFile);
			filter.fillMeasurements(filter.droneMeasurements[droneId], pose.center(), pose.rotation());
		}
		else {
			if (cov.size() == 0) {
				double cov_pose[6 * 6] = { 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1 };
				std::array<double, 6 * 6> covpose;
				std::copy(std::begin(cov_pose), std::end(cov_pose), std::begin(covpose));
				cov.push_back(covpose);
			}

			Pose3 failurePose = Pose3(Mat3::Identity(), Vec3::Zero());
			logger.logPoseCovtoFile(colocInterface.imageNumber, droneId, droneId, failurePose, cov, rmse, nTracks, poseFile);
		}

		filter.update(droneId, pose, rmse);
		logger.logPoseCovtoFile(colocInterface.imageNumber, droneId, droneId, pose, cov, rmse, nTracks, filtPoseFile);

		//currentPoses[droneId] = pose;
		//currentCov[droneId] = cov;
	}

	void interPoseEstimator(int sourceId, int destId)
	{
		//std::string poseFile = params.imageFolder + "poses.txt";
		std::string matchesFile = params.imageFolder + "matchesInter_" + std::to_string(colocInterface.imageNumber) + ".svg";

		// colocData tempScene;

		for (unsigned int i = 0; i < 2; ++i) {
			data.tempScene.views[i].reset(new View(data.filenames[i], i, 0, i, params.imageSize.first, params.imageSize.second));
		}

		//for (auto &region : data.regions)
		//	data.tempScene.regions.emplace_hint(tempScene.regions.end(), region.first, region.second.get());

		Pair interPosePair = std::make_pair <IndexT, IndexT>((IndexT)sourceId, (IndexT)destId);

		IndMatches pairMatches;
		matcher.computeMatchesPair(interPosePair, data.regions, pairMatches);

		//data.putativeMatches.insert({ interPosePair, std::move(pairMatches) });


		data.putativeMatches.at(interPosePair) = pairMatches;
		robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches, data.relativePoses);

		utils.drawMatches(params.imageSize, matchesFile, data.filenames[0], data.filenames[1], *data.regions[0].get(), *data.regions[1].get(), data.geometricMatches.at({ 0,1 }));

		std::cout << "Creating temporary map" << std::endl;
		coloc::Reconstructor tempReconstructor(params);
		coloc::Utils tempUtils;
		tempReconstructor.reconstructScene(true, data, currentPoses, 1.0, false);

		PoseRefiner refiner;
		float rmse;
		Cov6 cov;
		const Optimize_Options refinementOptions(Intrinsic_Parameter_Type::NONE, Extrinsic_Parameter_Type::ADJUST_ALL, Structure_Parameter_Type::ADJUST_ALL);
		bool locStatus = refiner.refinePose(data.tempScene, refinementOptions, rmse, cov);

		data.tempScene.s_root_path = params.imageFolder;

		bool isInter = true;
		mapReady = data.setupMapDatabase(isInter);

		std::vector <IndMatch> commonFeatures; 
		matcher.matchMapFeatures(data.mapRegions, data.interMapRegions, commonFeatures);

		Vec3 poseDiff = data.tempScene.poses[0].center() - data.scene.poses[0].center();
		Mat3 rotDiff = data.tempScene.poses[0].rotation();
		robustMatcher.matchMaps(data.mapRegions, data.interMapRegions, commonFeatures, poseDiff, rotDiff);
		std::string matchesFileMap = params.imageFolder + "matchesMap_" + std::to_string(destId) + "_" + std::to_string(imageNumber) + ".svg";

		utils.drawMatches(params.imageSize, matchesFileMap, data.keyframeNames[0], data.filenames[0], *data.mapRegions.get(), *data.interMapRegions.get(), commonFeatures);
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

		Pose3 pose = Pose3(data.tempScene.poses.at(1).rotation(), (currentPoses[sourceId].center() + data.tempScene.poses.at(1).center()));
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

		covIntOptimizer.computeFusedValues();

		Pose3 poseFused;
		Cov6 covFused;

		//utils.readPoseCovariance(covIntOptimizer.covFused, covFused);

		poseFused.center()[0] = covIntOptimizer.poseFused(0);
		poseFused.center()[1] = covIntOptimizer.poseFused(1);
		poseFused.center()[2] = covIntOptimizer.poseFused(2);

		logger.logPoseCovtoFile(imageNumber, destId, sourceId, poseFused, cov, rmse, tracks, filtPoseFile);
	}

	void ColoC::updateMap(std::vector <int> drones)
	{
		colocData updateData;
		
		std::string filename;
		std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);
		updateData.filenames.clear();
		for (unsigned int i = 0; i < drones.size(); ++i) {
			std::string filename = params.imageFolder + "img__Quad" + std::to_string(drones[i]) + "_" + number + ".png";
			updateData.filenames.push_back(filename);
			std::cout << updateData.filenames[i] << std::endl;

			detector.detectFeaturesFile(i, updateData.regions, updateData.filenames[i]);

			updateData.scene.views[i].reset(new View(updateData.filenames[i], i, 0, i, params.imageSize.first, params.imageSize.second));
		}
		
		matcher.computeMatches(updateData.regions, updateData.putativeMatches);
		robustMatcher.filterMatches(updateData.regions, updateData.putativeMatches, updateData.geometricMatches, updateData.relativePoses);

#ifdef DEBUG
		std::string matchesFile_putative = params.imageFolder + "matches_putative_" + std::to_string(updateNum) + ".svg";
		std::string matchesFile_geometric = params.imageFolder + "matches_geometric_" + std::to_string(updateNum) + ".svg";
		utils.drawMatches(params.imageSize, matchesFile_putative, updateData.filenames[0], updateData.filenames[1], *updateData.regions[0].get(), *updateData.regions[1].get(), updateData.putativeMatches.at({ 0,1 }));
		utils.drawMatches(params.imageSize, matchesFile_geometric, updateData.filenames[0], updateData.filenames[1], *updateData.regions[0].get(), *updateData.regions[1].get(), updateData.geometricMatches.at({ 0,1 }));
#endif

		std::cout << "Updating map" << std::endl;
		Reconstructor updateReconstructor(params);
		updateReconstructor.reconstructScene(0, updateData, currentPoses, 3.0, true);

		std::string newMapFile = params.imageFolder + "newmap_" + std::to_string(updateNum) + ".ply";
		updateNum++;
		logger.logMaptoPLY(updateData.scene, newMapFile);

		bool newMapReady = updateData.setupMapDatabase(0);
#ifdef DEBUG		
		std::string mapFeatFile = params.imageFolder + "UpdatedMap_Features.svg";
		utils.drawFeatures(updateData.filenames[0], params.imageSize, updateData.mapRegions->Features(), mapFeatFile);
#endif

		if (newMapReady == EXIT_SUCCESS) {
			std::vector <IndMatch> commonFeatures;

			std::string mapmatchesFile = params.imageFolder + "mapmatches_" + std::to_string(updateNum) + ".svg";
			matcher.matchMapFeatures(data.mapRegions, updateData.mapRegions, commonFeatures); 

			Vec3 poseDiff = updateData.scene.poses[0].center() - data.scene.poses[0].center();
			Mat3 rotDiff = updateData.scene.poses[0].rotation() - data.scene.poses[0].rotation();
			robustMatcher.matchMaps(data.mapRegions, updateData.mapRegions, commonFeatures, poseDiff, updateData.scene.poses[0].rotation());
			double scaleDiff = utils.computeScaleDifference(data.scene, data.mapRegionIdx, updateData.scene, updateData.mapRegionIdx, commonFeatures);
			std::cout << "Scale factor ratio computed during update as " << scaleDiff << std::endl;
			utils.rescaleMap(updateData.scene, scaleDiff);

#ifdef DEBUG
			utils.drawMatches(params.imageSize, mapmatchesFile, data.keyframeNames[0], updateData.filenames[0], *data.mapRegions.get(), *updateData.mapRegions.get(), commonFeatures);
#endif
		}
		
		data = updateData;
		data.setupMapDatabase(0);

#ifdef USE_CUDA
		matcher.setMapData(data.mapRegions->RegionCount(), const_cast<unsigned int*>(static_cast<const unsigned int*>(data.mapRegions->DescriptorRawData())));
#endif
	}
};