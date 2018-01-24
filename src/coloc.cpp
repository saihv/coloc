#pragma once

#include "coloc.hpp"

using namespace coloc;

LocalizationParams initParams()
{
	LocalizationParams params;
	params.featureDetectorType = "BINARY";
	params.imageSize = std::make_pair(640, 480);
	params.K << 320, 0, 320, 0, 320, 240, 0, 0, 1;
	params.imageFolder = "C:/Users/saihv/Desktop/3traj/";
	params.filterType = 'H';
	return params;
}

int main()
{
	int numDrones = 3;
	int startNum = 0;
	std::vector<int> drones;
	LocalizationParams params = initParams();
	drones.push_back(0);
	drones.push_back(1);
	drones.push_back(2);
	ColoC coloc(numDrones, startNum, params);

	coloc.poseFile = params.imageFolder + "poses.txt";
	coloc.mapFile = params.imageFolder + "output.ply";
	if (std::experimental::filesystem::exists(coloc.poseFile)) {
		if (remove(coloc.poseFile.c_str()) == 0) {
			std::cout << "Removed old log files" << std::endl;
		}
		else {
			std::cout << "Can't remove old log files" << coloc.poseFile << ": "
				<< strerror(errno) << std::endl;

			getchar();
			return -1;
		}
	}

	coloc.initMap(5.0);

	Pose3 pose;
	Cov6 cov;
	Plotter plotter;
	numDrones = 3;
	plotter.plotScene(coloc.data.scene);
	coloc.imageNumber = 1;
	while (coloc.imageNumber < 193) {

		for (int droneId = 0; droneId < numDrones; droneId++) {
			Pose3 poseIntra;
			Cov6 covIntra;
			coloc.intraPoseEstimator(droneId, poseIntra, covIntra);
			plotter.plotPose(poseIntra, 0);
		}

		if (coloc.imageNumber % 10 == 0)
		{
			Pose3 intraPose;
			Cov6 intraCov;
			coloc.interPoseEstimator(0, 1, Pose3(), intraPose, intraCov);
			coloc.interPoseEstimator(0, 2, Pose3(), intraPose, intraCov);
		}

		coloc.imageNumber++;
	}

	/*
	coloc.imageNumber = 10;
	coloc.interPoseEstimator(0, 1, Pose3(Mat3::Identity(), Vec3::Zero()), pose, cov);	

	std::vector <int> drones;
	drones.push_back(2);
	drones.push_back(3);
	coloc.updateMap(drones);
	
	plotter.plotScene(coloc.data.scene);
	
	int droneId = 1;
	coloc.imageNumber = 250;
	for (int i = 250; i < 481; ++i) {
		if (i == 251) {
			std::vector <int> drones;
			drones.push_back(2);
			drones.push_back(3);
			coloc.updateMap(drones);
		}

		coloc.intraPoseEstimator(droneId, poseIntra, covIntra);
		plotter.plotPose(poseIntra);
		coloc.imageNumber++;
	}

	coloc.interPoseEstimator(0, 2, coloc.data.scene.poses.at(0), poseInter, covInter);
	
	plotter.plotPoseandCovariance(poseIntra, covIntra);
	plotter.plotPoseandCovariance(poseInter, covInter);
	*/
	plotter.drawPlot();
	
	getchar();
	return 0;
}

