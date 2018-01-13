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
		Localizer(LocalizationParams& params)
		{
			if (params.featureDetectorType == "SIFT")
				image_describer.reset(new features::SIFT_Image_describer(features::SIFT_Image_describer::Params(), true));
			else if (params.featureDetectorType == "AKAZE")
				image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MSURF), true);

			this->imageSize = &params.imageSize;
			this->K = &params.K;
			this->rootFolder = &params.imageFolder;
		}

		std::unique_ptr<features::Regions> regionsCurrent;
		bool localizeImage(std::string& imageName, geometry::Pose3& pose, LocalizationData &data);
		bool matchLocalize(LocalizationData &data, const features::Regions & queryRegions, geometry::Pose3 & pose, Image_Localizer_Match_Data * trackPtr);
		bool refine(Pose3& pose, Image_Localizer_Match_Data& matchData);
		bool setupMap(LocalizationData& data);
		bool initMatchingInterface(SfM_Data& scene, Regions_Provider& mapFeatures);
		
	private:
		std::unique_ptr<features::Image_describer> image_describer;
		std::pair <size_t, size_t> *imageSize;
		Mat3 *K;
		std::string *rootFolder;

		SfM_Data *scenePtr;

		std::unique_ptr<Regions> regions_type;
		std::shared_ptr<Regions_Provider> mapFeatures = std::make_shared<Regions_Provider>();

		std::shared_ptr<matching::Matcher_Regions_Database> matchInterface;


		std::unique_ptr<features::Regions> mapDesc;
		std::vector<IndexT> mapDescIdx;
	};

	bool Localizer::matchLocalize(LocalizationData &data, const features::Regions & queryRegions, geometry::Pose3 & pose, Image_Localizer_Match_Data * trackPtr)
	{
		const openMVG::cameras::Pinhole_Intrinsic cam(imageSize->first, imageSize->second, (*K)(0, 0), (*K)(0, 2), (*K)(1, 2));
		
		if (scenePtr == nullptr)
			return EXIT_FAILURE;

		std::vector<IndMatch> trackedFeatures;
		//if (!this->matchInterface->Match(0.8, queryRegions, trackedFeatures))

		matching::DistanceRatioMatch(
			0.8, matching::CASCADE_HASHING_L2,
			*data.mapRegions.get(),
			queryRegions,
			trackedFeatures);

		if (trackedFeatures.empty())
		{
			std::cout << "Unable to track any features" << std::endl;
			return EXIT_FAILURE;
		}

		std::cout << "Number of tracked features: " << trackedFeatures.size() << std::endl;

		Image_Localizer_Match_Data trackData;
		if (trackPtr)
			trackData.error_max = trackPtr->error_max;
		
		trackData.pt3D.resize(3, trackedFeatures.size());
		trackData.pt2D.resize(2, trackedFeatures.size());
		Mat2X pt2D_original(2, trackedFeatures.size());

		scenePtr = &data.scene;

		for (size_t i = 0; i < trackedFeatures.size(); ++i)
		{
			trackData.pt3D.col(i) = scenePtr->GetLandmarks().at(data.mapRegionIdx[trackedFeatures[i].i_]).X;
			trackData.pt2D.col(i) = queryRegions.GetRegionPosition(trackedFeatures[i].j_);
			pt2D_original.col(i) = trackData.pt2D.col(i);

			if (&cam && cam.have_disto())
				trackData.pt2D.col(i) = cam.get_ud_pixel(trackData.pt2D.col(i));
		}
		
		bool localizationStatus = SfM_Localizer::Localize(resection::SolverType::P3P_KE_CVPR17, *imageSize, &cam, trackData, pose);
		trackData.pt2D = std::move(pt2D_original);

		if (trackPtr)
			(*trackPtr) = std::move(trackData);

		return localizationStatus;
	}
	
	bool Localizer::localizeImage(std::string& imageName, geometry::Pose3& pose, LocalizationData &data)
	{
		using namespace openMVG::features;

		image::Image<unsigned char> imageGray;

		if (!ReadImage(imageName.c_str(), &imageGray)) {
			std::cout << "Unable to read image from the given path." << std::endl;
		}
		image_describer->Describe(imageGray, regionsCurrent);
		std::cout << "#regions detected in query image: " << regionsCurrent->RegionCount() << std::endl;

		sfm::Image_Localizer_Match_Data matching_data;
		matching_data.error_max = std::numeric_limits<double>::infinity();

		if (!this->matchLocalize(data, *regionsCurrent.get(), pose, &matching_data)) {
			std::cout << "Localization unsuccessful" << std::endl;
			return EXIT_FAILURE;
		}
		else {
			std::cout << "Localization successful" << std::endl;
			//refine(pose, matching_data);
			std::shared_ptr<cameras::IntrinsicBase> optional_intrinsic;
			optional_intrinsic = std::make_shared<cameras::Pinhole_Intrinsic_Radial_K3>(
				imageGray.Width(), imageGray.Height(),
				(*K)(0, 0), (*K)(0, 2), (*K)(1, 2));
			//const openMVG::cameras::Pinhole_Intrinsic cam(imageSize->first, imageSize->second, (*K)(0, 0), (*K)(0, 2), (*K)(1, 2));
			if (this->refine(pose, matching_data))
			{
				std::cerr << "Refining pose for image failed." << std::endl;
			}
			return EXIT_SUCCESS;
		}

	}

	bool Localizer::refine(Pose3& pose, Image_Localizer_Match_Data& matchData)
	{
		SfM_Data scene;
		scene.views.insert({ 0, std::make_shared<View>("",0, 0, 0) });
		scene.poses[0] = pose;
		const openMVG::cameras::Pinhole_Intrinsic cam(imageSize->first, imageSize->second, (*K)(0, 0), (*K)(0, 2), (*K)(1, 2));
		std::shared_ptr<cameras::IntrinsicBase> shared_intrinsics(cam.clone());
		scene.intrinsics[0] = shared_intrinsics;
		// structure data (2D-3D correspondences)
		for (size_t i = 0; i < matchData.vec_inliers.size(); ++i)
		{
			const size_t idx = matchData.vec_inliers[i];
			Landmark landmark;
			landmark.X = matchData.pt3D.col(idx);
			landmark.obs[0] = Observation(matchData.pt2D.col(idx), UndefinedIndexT);
			scene.structure[i] = std::move(landmark);
		}

		// Configure BA options (refine the intrinsic and the pose parameter only if requested)
		const Optimize_Options ba_refine_options
		(cameras::Intrinsic_Parameter_Type::NONE, Extrinsic_Parameter_Type::ADJUST_ALL, Structure_Parameter_Type::NONE);
		PoseRefiner refiner;
		const bool b_BA_Status = refiner.refinePose(
			scene,
			ba_refine_options);
		if (b_BA_Status)
		{
			pose = scene.poses[0];
		}

		return b_BA_Status;
	}
}
