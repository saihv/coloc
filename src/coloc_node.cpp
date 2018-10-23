#pragma once

#include "coloc/coloc.hpp"

using namespace openMVG;

void readCalibData(std::vector<Mat3>& K, std::vector<Vec3>& dist, std::string& fileName, unsigned int& numDrones)
{
	std::ifstream calibData;
	calibData.open(fileName);

	std::string line;
	std::vector<double> values;
	uint rows = 0;
	while (rows < numDrones) {
		std::getline(calibData, line);
		std::stringstream lineStream(line);
		std::string cell;
		while (std::getline(lineStream, cell, ',')) {
			values.push_back(std::stod(cell));
		}
		Eigen::Matrix3d copiedMatrix = Eigen::Map<Eigen::Matrix3d>(&values[0], 3, 3);
		K.push_back(copiedMatrix);
		values.clear();
		++rows;
	}
	rows = 0;
	while (rows < numDrones) {
		std::getline(calibData, line);
		std::stringstream lineStream(line);
		std::string cell;
		while (std::getline(lineStream, cell, ',')) {
			values.push_back(std::stod(cell));
		}
		Eigen::Vector3d copiedMatrix = Eigen::Map<Eigen::Vector3d>(&values[0], 3, 1);
		dist.push_back(copiedMatrix);
		values.clear();
		++rows;
	}
}

int main(int argc, char **argv)
{
#ifdef USE_ROS
		ros::init(argc, argv, "coloc");
		ros::NodeHandle nh;
#endif

		DetectorOptions Dopts;
		MatcherOptions Mopts;

		Dopts.width = 1280;
		Dopts.height = 720;
		Dopts.maxkp = 20000;
		Dopts.scale_factor = 1.2;
		Dopts.scale_levels = 8;
		Dopts.thresh = 20;

		Mopts.distRatio = 0.8;
		Mopts.maxkp = 20000;
		Mopts.thresh = 20;

		unsigned int numDrones = 3;

		std::vector <Mat3> K;
		K.reserve(numDrones);
		std::vector <Vec3> dist;
		dist.reserve(numDrones);

		Mat3 tempK;
		Vec3 tempdist;

		std::string imageFolder = "C:\\Users\\Sai\\Desktop\\testRellis\\";
		std::string calibFilename = imageFolder + "calib.txt";

		readCalibData(K, dist, calibFilename, numDrones);

		coloc::colocParams params(K, dist, 'H', std::make_pair <size_t, size_t>(1280, 720), imageFolder, Dopts, Mopts);

		int nStart = 0;
		ColoC coloc(numDrones, nStart, params, Dopts, Mopts);

		coloc.mainThread();

		return 0;
}
