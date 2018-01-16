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
#include "localizationUtils.hpp"
#include "estimationKernel.hpp"
#include <filesystem>

using namespace coloc;

int main()
{
	std::string filename;
	LocalizationParams params;
	params.featureDetectorType = "BINARY";
	params.imageSize = std::make_pair(640, 480);
	params.K << 320, 0, 320, 0, 320, 240, 0, 0, 1;
	params.imageFolder = "C:/Users/saihv/Desktop/updatetest/";

	FeatureExtractor detector(params);
	FeatureMatcher matcher(params);
	RobustMatcher robustMatcher(params);
	Reconstructor reconstructor(params), recNew(params);
	Localizer localizer(params);
	Logger logger;
	Utils utils, utils2;

	LocalizationData data1, data2;
	
	std::string filename1 = params.imageFolder + "img__Quad" + std::to_string(0) + "_" + "0000" + ".png";
	std::string filename2 = params.imageFolder + "img__Quad" + std::to_string(1) + "_" + "0000" + ".png";
	detector.detectFeatures(0, data1.regions, filename1);
	detector.detectFeatures(1, data1.regions, filename2);
	detector.saveFeatureData(0, data1.regions, filename1);
	detector.saveFeatureData(1, data1.regions, filename2);
	data1.scene.views[0].reset(new View(filename1, 0, 0, 0, params.imageSize.first, params.imageSize.second));
	data1.scene.views[1].reset(new View(filename2, 1, 0, 1, params.imageSize.first, params.imageSize.second));
	matcher.computeMatches(data1.regions, data1.putativeMatches);
	robustMatcher.filterMatches(data1.regions, data1.putativeMatches, data1.geometricMatches, data1.relativePoses);
	data1.scene.s_root_path = params.imageFolder;
	reconstructor.reconstructScene(data1, Pose3(Mat3::Identity(), Vec3::Zero()), 1.0, true);
	utils.setupMap(data1, params);

	std::string filename3 = params.imageFolder + "img__Quad" + std::to_string(0) + "_" + "0010" + ".png";
	std::string filename4 = params.imageFolder + "img__Quad" + std::to_string(1) + "_" + "0010" + ".png";
	detector.detectFeatures(0, data2.regions, filename3);
	detector.detectFeatures(1, data2.regions, filename4);
	detector.saveFeatureData(0, data2.regions, filename3);
	detector.saveFeatureData(1, data2.regions, filename4);
	data2.scene.views[0].reset(new View(filename3, 0, 0, 0, params.imageSize.first, params.imageSize.second));
	data2.scene.views[1].reset(new View(filename4, 1, 0, 1, params.imageSize.first, params.imageSize.second));
	matcher.computeMatches(data2.regions, data2.putativeMatches);
	robustMatcher.filterMatches(data2.regions, data2.putativeMatches, data2.geometricMatches, data2.relativePoses);
	data2.scene.s_root_path = params.imageFolder;
	recNew.reconstructScene(data2, Pose3(Mat3::Identity(), Vec3::Zero()), 1.0, true);
	utils2.setupMap(data2, params);

	std::vector <IndMatch> putativeMatches, filteredMatches, geometricMatches;

	DistanceRatioMatch(0.8, BRUTE_FORCE_HAMMING, *data1.mapRegions.get(), *data2.mapRegions.get(), putativeMatches);
	utils.filterMapMatches(params, *data1.mapRegions.get(), *data2.mapRegions.get(), putativeMatches, geometricMatches);
	filterHomography(params, *data1.mapRegions.get(), *data2.mapRegions.get(), { 0,1 }, putativeMatches, filteredMatches);

	std::string drawPutative = "putativeMatches.svg";
	std::string drawGeometric = "geometricMatches.svg";
	std::string drawFiltered = "filteredMatches.svg";

	utils2.drawMatches(drawPutative, filename1, filename3, *data1.mapRegions.get(), *data2.mapRegions.get(), putativeMatches);
	utils2.drawMatches(drawFiltered, filename1, filename3, *data1.mapRegions.get(), *data2.mapRegions.get(), filteredMatches);
	utils2.drawMatches(drawGeometric, filename1, filename3, *data1.mapRegions.get(), *data2.mapRegions.get(), geometricMatches);

	Mat pt3D_1, pt3D_2;

	pt3D_1.resize(3, filteredMatches.size());
	pt3D_2.resize(3, filteredMatches.size());

	float scale = 0.0;

	for (size_t i = 0; i < filteredMatches.size(); ++i)
	{
		pt3D_1.col(i) = data1.scene.GetLandmarks().at(data1.mapRegionIdx[filteredMatches[i].i_]).X;
		pt3D_2.col(i) = data2.scene.GetLandmarks().at(data2.mapRegionIdx[filteredMatches[i].j_]).X;
	}

	for (size_t i = 0; i < filteredMatches.size() - 1; ++i)
	{
		Vec3 X11 = data1.scene.GetLandmarks().at(data1.mapRegionIdx[filteredMatches[i].i_]).X;
		Vec3 X12 = data1.scene.GetLandmarks().at(data1.mapRegionIdx[filteredMatches[i + 1].i_]).X;

		Vec3 X21 = data2.scene.GetLandmarks().at(data2.mapRegionIdx[filteredMatches[i].j_]).X;
		Vec3 X22 = data2.scene.GetLandmarks().at(data2.mapRegionIdx[filteredMatches[i + 1].j_]).X;

		float dist1 = (X12 - X11).norm();
		float dist2 = (X22 - X21).norm();

		scale += std::max(dist1, dist2) / std::min(dist1, dist2);
	}
	float scaleDiff = scale / (filteredMatches.size() - 1);

	utils2.rescaleMap(data2.scene, scaleDiff);

	std::string mapOriginal = "original.ply";
	std::string mapFinal = "scaled.ply";

	logger.logMaptoPLY(data2.scene, mapOriginal);
	logger.logMaptoPLY(data2.scene, mapFinal);

	getchar();
}