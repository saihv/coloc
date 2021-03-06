#pragma once

#include "Refiner.hpp"
#include "colocUtils.hpp"
#include "coloc/colocParams.hpp"
#include "coloc/colocData.hpp"

#include "opencv2/core.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::sfm;

namespace coloc
{
	class Localizer {
	public:
		explicit Localizer(colocParams& params)
		{
			image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MLDB), true);
			matchingType = matching::BRUTE_FORCE_HAMMING;
			useGPU = false;

			this->imageSize = &params.imageSize;
			this->K = &params.K;
			this->dist = &params.dist;
			this->rootFolder = &params.imageFolder;
		}

		std::unique_ptr<features::Regions> regionsCurrent;
		bool localizeImage(int&, Pose3&, colocData&, Cov6&, float&, IndMatches&, std::vector<uint32_t>&);
		bool setupTracks(cameras::Pinhole_Intrinsic_Radial_K3* cam, colocData &data, const features::Regions & queryRegions, IndMatches &trackedFeatures, Image_Localizer_Match_Data * trackPtr);
		bool refine(int&, Pose3&, Image_Localizer_Match_Data&, Cov6&, float&);

	private:
		std::unique_ptr<features::Image_describer> image_describer;
		EMatcherType matchingType;
		std::pair <int, int> *imageSize;
		std::vector <Mat3> *K;
		std::vector <Vec3> *dist;
		std::string *rootFolder;

		Scene *scenePtr;

		std::unique_ptr<Regions> regions_type;
		std::shared_ptr<Regions_Provider> mapFeatures = std::make_shared<Regions_Provider>();

		std::shared_ptr<matching::Matcher_Regions_Database> matchInterface;

		bool useGPU;
		std::unique_ptr<features::Regions> mapDesc;
		std::vector<IndexT> mapDescIdx;
	};

	bool Localizer::setupTracks(cameras::Pinhole_Intrinsic_Radial_K3* cam, colocData &data, const features::Regions & queryRegions, IndMatches &trackedFeatures, Image_Localizer_Match_Data * trackPtr)
	{
		trackPtr->pt3D.resize(3, trackedFeatures.size());
		trackPtr->pt2D.resize(2, trackedFeatures.size());
		Mat2X pt2D_original(2, trackedFeatures.size());

		for (size_t i = 0; i < trackedFeatures.size(); ++i) {
			trackPtr->pt3D.col(i) = data.scene.GetLandmarks().at(data.mapRegionIdx[trackedFeatures[i].i_]).X;
			trackPtr->pt2D.col(i) = queryRegions.GetRegionPosition(trackedFeatures[i].j_);
			pt2D_original.col(i) = trackPtr->pt2D.col(i);

			if (&cam && cam->have_disto())
				trackPtr->pt2D.col(i) = cam->get_ud_pixel(trackPtr->pt2D.col(i));
		}

		return EXIT_SUCCESS;
	}

	bool Localizer::localizeImage(int& idx, Pose3& pose, colocData &data, Cov6 &covariance, float& rmse, IndMatches &trackedFeatures, std::vector<uint32_t>& inliers)
	{
		using namespace openMVG::features;
		openMVG::cameras::Pinhole_Intrinsic_Radial_K3 cam(imageSize->first, imageSize->second, (*K)[idx](0, 0), (*K)[idx](0, 2), (*K)[idx](1, 2), (*dist)[idx](0), (*dist)[idx](1), (*dist)[idx](2));
		
		Image_Localizer_Match_Data matching_data;
		matching_data.error_max = std::numeric_limits<double>::infinity();
		matching_data.max_iteration = 256;

		Image_Localizer_Match_Data trackData;

		if (setupTracks(&cam, data, *data.regions.at(idx).get(), trackedFeatures, &matching_data) == EXIT_FAILURE) {
			std::cout << "Failure while setting up 2D-3D correspondences" << std::endl;
			return EXIT_FAILURE;
		}
		else {
			bool localizationStatus = SfM_Localizer::Localize(resection::SolverType::P3P_KE_CVPR17, *imageSize, &cam, matching_data, pose);
			if (!localizationStatus) {
				std::cout << "Localization unsuccessful" << std::endl;
				return EXIT_FAILURE;
			}
			else {
				std::cout << "Localization successful" << std::endl;
				inliers = matching_data.vec_inliers;
				if (!this->refine(idx, pose, matching_data, covariance, rmse))
					std::cerr << "Refining pose for image failed." << std::endl;

				return EXIT_SUCCESS;
			}
		}

	}

	bool Localizer::refine(int &idx, Pose3 &pose, Image_Localizer_Match_Data& matchData, Cov6 &poseCovariance, float& rmse)
	{
		Scene scene;

		std::vector <cv::Point3f> objectPoints;
		std::vector <cv::Point2f> imagePoints;
		scene.views.insert({ 0, std::make_shared<View>("",0, 0, 0) });
		scene.poses[0] = pose;
		const openMVG::cameras::Pinhole_Intrinsic_Radial_K3 cam(imageSize->first, imageSize->second, (*K)[idx](0, 0), (*K)[idx](0, 2), (*K)[idx](1, 2), (*dist)[idx](0), (*dist)[idx](1), (*dist)[idx](2));
		std::shared_ptr<cameras::IntrinsicBase> shared_intrinsics(cam.clone());
		scene.intrinsics[0] = shared_intrinsics;

		for (size_t i = 0; i < matchData.vec_inliers.size(); ++i) {
			const size_t idx = matchData.vec_inliers[i];
			Landmark landmark;
			landmark.X = matchData.pt3D.col(idx);
			objectPoints.push_back(cv::Point3f(landmark.X[0], landmark.X[1], landmark.X[2]));
			landmark.obs[0] = Observation(matchData.pt2D.col(idx), UndefinedIndexT);
			imagePoints.push_back(cv::Point2f(matchData.pt2D.col(idx)[0], matchData.pt2D.col(idx)[1]));
			scene.structure[i] = std::move(landmark);
		}

		const Optimize_Options ba_refine_options
		(cameras::Intrinsic_Parameter_Type::NONE, Extrinsic_Parameter_Type::ADJUST_ALL, Structure_Parameter_Type::NONE);
		PoseRefiner refiner;
		const bool refineStatus = refiner.refinePose(
			scene,
			ba_refine_options, rmse, poseCovariance);
		if (refineStatus)
			pose = scene.poses[0];

		cv::Mat rvec, J;

		cv::eigen2cv(pose.rotation(), rvec);
		cv::Mat tvec = cv::Mat(1, 3, CV_32FC1, cv::Scalar::all(0));

		tvec.at<float>(0, 0) = pose.translation()[0];
		tvec.at<float>(0, 1) = pose.translation()[1];
		tvec.at<float>(0, 2) = pose.translation()[2];

		cv::Mat Kcv = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
		Kcv.at<float>(0, 0) = (*K)[idx](0, 0);
		Kcv.at<float>(0, 2) = (*K)[idx](0, 2);
		Kcv.at<float>(1, 1) = (*K)[idx](0, 0);
		Kcv.at<float>(1, 2) = (*K)[idx](1, 2);
		Kcv.at<float>(2, 2) = 1.0f;

		cv::Mat distcv = cv::Mat(5, 1, CV_32FC1, cv::Scalar::all(0));


		std::vector <cv::Point2f> reprojected;
		cv::projectPoints(objectPoints, rvec, tvec, Kcv, distcv, reprojected, J);

		
		float error = 0.0;
		for (int j = 0; j < reprojected.size(); j++)
		{
			error += cv::norm(imagePoints[j] - reprojected[j]);
		}

		rmse = error / reprojected.size();

		cv::Mat Sigma = cv::Mat(J.t() * J, cv::Rect(0, 0, 6, 6)).inv();
		cv::Mat std_dev;
		sqrt(Sigma.diag(), std_dev);

		return refineStatus;
	}
}
