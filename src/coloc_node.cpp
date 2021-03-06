#include "coloc/coloc.hpp"

using namespace openMVG;

void readCalibData(std::pair <int, int>& imageSize, std::vector<Mat3>& K, std::vector<Vec3>& dist, std::string& fileName, unsigned int& numDrones)
{
	std::ifstream calibData;
	calibData.open(fileName);

	std::string line;
	std::vector<double> values;

	std::getline(calibData, line);
	std::stringstream lineStream(line);
	std::string cell;
	while (std::getline(lineStream, cell, ',')) {
		values.push_back(std::stod(cell));
	}

	imageSize.first = values[0];
	imageSize.second = values[1];

	values.clear();
	uint rows = 0;
	while (rows < numDrones) {
		std::getline(calibData, line);
		std::stringstream lineStream(line);
		std::string cell;
		while (std::getline(lineStream, cell, ',')) {
			values.push_back(std::stod(cell));
		}

		Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > copiedMatrix(&values[0]);
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
		unsigned int numDrones = 2;

		Mat3 tempK;
		Vec3 tempdist;

		std::pair <int, int> imageSize;
		std::vector <Mat3> K;
		std::vector <Vec3> dist;

		std::string imageFolder = "/home/sai/sampleColoc/";
		std::string calibFilename = imageFolder + "calib.txt";

		readCalibData(imageSize, K, dist, calibFilename, numDrones);

		DetectorOptions Dopts;
		MatcherOptions Mopts;

		Dopts.width = imageSize.first;
		Dopts.height = imageSize.second;
		Dopts.maxkp = 5000;
		Dopts.scale_factor = 1.2;
		Dopts.scale_levels = 8;
		Dopts.thresh = 40;

		Mopts.distRatio = 0.8;
		Mopts.maxkp = 5000;
		Mopts.thresh = 60;

		coloc::colocParams params(K, dist, 'E', std::make_pair(imageSize.first, imageSize.second), imageFolder, Dopts, Mopts);

		int nStart = 0;
		ColoC coloc(numDrones, nStart, params, Dopts, Mopts);

		coloc.mainThread();

		return 0;
}
