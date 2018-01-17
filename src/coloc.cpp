#pragma once

#include "coloc.hpp"

using namespace coloc;

LocalizationParams initParams()
{
	LocalizationParams params;
	params.featureDetectorType = "BINARY";
	params.imageSize = std::make_pair(640, 480);
	params.K << 320, 0, 320, 0, 320, 240, 0, 0, 1;
	params.imageFolder = "C:/Users/saihv/Desktop/testnewer/";
	params.filterType = 'H';
	return params;
}

int main()
{
	int numDrones = 2;
	int startNum = 0;
	LocalizationParams params = initParams();

	ColoC coloc(numDrones, startNum, params);

	coloc.poseFile = params.imageFolder + "poses.txt";
	coloc.mapFile = params.imageFolder + "output.ply";

	coloc.initMap(1.0);

	Pose3 pose;
	Cov6 cov;
	//coloc.imageNumber = 10;
	//coloc.interPoseEstimator(0, 1, Pose3(Mat3::Identity(), Vec3::Zero()), pose, cov);
	/*
	

	std::vector <int> drones;
	drones.push_back(2);
	drones.push_back(3);
	coloc.updateMap(drones);
	
	plotter.plotScene(coloc.data.scene);
	*/

	Pose3 poseIntra, poseInter;
	Cov6 covIntra, covInter;

	Plotter plotter;
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

	//coloc.interPoseEstimator(0, 2, coloc.data.scene.poses.at(0), poseInter, covInter);
	
	//plotter.plotPoseandCovariance(poseIntra, covIntra);
	//plotter.plotPoseandCovariance(poseInter, covInter);
	plotter.drawPlot();
	
	getchar();
	return 0;
}

