#pragma once

#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/image_describer_akaze_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "nonFree/sift/SIFT_describer_io.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "poseRefiner.hpp"
#include "localizationUtils.hpp"

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
		explicit Localizer(LocalizationParams& params)
		{
			if (params.featureDetectorType == "SIFT") {
				image_describer.reset(new features::SIFT_Image_describer(features::SIFT_Image_describer::Params(), true));
				matchingType = CASCADE_HASHING_L2;
			}
			else if (params.featureDetectorType == "AKAZE") {
				image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MSURF), true);
				matchingType = BRUTE_FORCE_L2;
			}
			else if (params.featureDetectorType == "BINARY") {
				image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MLDB), true);
				matchingType = BRUTE_FORCE_HAMMING;
			}

			this->imageSize = &params.imageSize;
			this->K = &params.K;
			this->rootFolder = &params.imageFolder;
		}

		std::unique_ptr<features::Regions> regionsCurrent;
		bool localizeImage(std::string&, Pose3&, LocalizationData&, Cov6&);
		bool matchSceneWithMap(IntrinsicBase* cam, LocalizationData &data, const features::Regions & queryRegions, Image_Localizer_Match_Data * trackPtr);
		bool refine(Pose3&, Image_Localizer_Match_Data&, Cov6&);
		
	private:
		std::unique_ptr<features::Image_describer> image_describer;
		EMatcherType matchingType;
		std::pair <size_t, size_t> *imageSize;
		Mat3 *K;
		std::string *rootFolder;

		Scene *scenePtr;

		std::unique_ptr<Regions> regions_type;
		std::shared_ptr<Regions_Provider> mapFeatures = std::make_shared<Regions_Provider>();

		std::shared_ptr<matching::Matcher_Regions_Database> matchInterface;


		std::unique_ptr<features::Regions> mapDesc;
		std::vector<IndexT> mapDescIdx;
	};

	bool Localizer::matchSceneWithMap(IntrinsicBase* cam, LocalizationData &data, const features::Regions & queryRegions, Image_Localizer_Match_Data * trackPtr)
	{
		if (&data.scene == nullptr) {
			std::cout << "No existing map. Localization cannot continue." << std::endl;
			return Failure;
		}

		std::vector<IndMatch> trackedFeatures;

		matching::DistanceRatioMatch(
			0.8, matchingType,
			*data.mapRegions.get(),
			queryRegions,
			trackedFeatures);

		if (trackedFeatures.empty()) {
			std::cout << "Unable to track any features" << std::endl;
			return Failure;
		}

		std::cout << "Number of tracked features: " << trackedFeatures.size() << std::endl;

		//trackPtr.error_max = trackPtr->error_max;
		
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
		
		//trackPtr->pt2D = std::move(pt2D_original);

		//if (trackPtr)
		//	(*trackPtr) = std::move(trackData);

		return Success;
	}
	
	bool Localizer::localizeImage(std::string& imageName, Pose3& pose, LocalizationData &data, Cov6 &covariance)
	{
		using namespace openMVG::features;
		openMVG::cameras::Pinhole_Intrinsic cam(imageSize->first, imageSize->second, (*K)(0, 0), (*K)(0, 2), (*K)(1, 2));
		image::Image<unsigned char> imageGray;

		if (!ReadImage(imageName.c_str(), &imageGray)) {
			std::cout << "Unable to read image from the given path." << std::endl;
		}
		image_describer->Describe(imageGray, regionsCurrent);
		std::cout << "#regions detected in query image: " << regionsCurrent->RegionCount() << std::endl;

		Image_Localizer_Match_Data matching_data;
		matching_data.error_max = std::numeric_limits<double>::infinity();

		Image_Localizer_Match_Data trackData;

		if (!this->matchSceneWithMap(&cam, data, *regionsCurrent.get(), &matching_data)) {
			std::cout << "Unable to match view with existing map" << std::endl;
			return Failure;
		}
		else {
			bool localizationStatus = SfM_Localizer::Localize(resection::SolverType::P3P_KE_CVPR17, *imageSize, &cam, matching_data, pose);
			if (!localizationStatus) {
				std::cout << "Localization unsuccessful" << std::endl;
				return Failure;
			}
			else {
				std::cout << "Localization successful" << std::endl;

				if (!this->refine(pose, matching_data, covariance))
					std::cerr << "Refining pose for image failed." << std::endl;

				return Success;
			}
		}

	}

	bool Localizer::refine(Pose3 &pose, Image_Localizer_Match_Data& matchData, Cov6 &poseCovariance)
	{
		Scene scene;
		scene.views.insert({ 0, std::make_shared<View>("",0, 0, 0) });
		scene.poses[0] = pose;
		const openMVG::cameras::Pinhole_Intrinsic cam(imageSize->first, imageSize->second, (*K)(0, 0), (*K)(0, 2), (*K)(1, 2));
		std::shared_ptr<cameras::IntrinsicBase> shared_intrinsics(cam.clone());
		scene.intrinsics[0] = shared_intrinsics;
		
		for (size_t i = 0; i < matchData.vec_inliers.size(); ++i) {
			const size_t idx = matchData.vec_inliers[i];
			Landmark landmark;
			landmark.X = matchData.pt3D.col(idx);
			landmark.obs[0] = Observation(matchData.pt2D.col(idx), UndefinedIndexT);
			scene.structure[i] = std::move(landmark);
		}

		const Optimize_Options ba_refine_options
		(cameras::Intrinsic_Parameter_Type::NONE, Extrinsic_Parameter_Type::ADJUST_ALL, Structure_Parameter_Type::NONE);
		PoseRefiner refiner;
		const bool refineStatus = refiner.refinePose(
			scene,
			ba_refine_options, poseCovariance);
		if (refineStatus)
			pose = scene.poses[0];
		
		return refineStatus;
	}
}
