#pragma once

#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/image_describer_akaze_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/descriptor.hpp"
#include "openMVG/features/feature.hpp"
#include "nonFree/sift/SIFT_describer_io.hpp"
#include "openMVG/sfm/sfm.hpp"

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
				image_describer = features::AKAZE_Image_describer::create(features::AKAZE_Image_describer::Params(features::AKAZE::Params(), features::AKAZE_MLDB), true);

			this->imageSize = &params.imageSize;
			this->K = &params.K;
			this->rootFolder = &params.imageFolder;
		}

		std::unique_ptr<features::Regions> regionsCurrent;
		bool localizeImage(std::string& imageName, geometry::Pose3& pose);
		bool setupMap(LocalizationData&);
		bool initMatchingInterface(SfM_Data&, Regions_Provider&);
		bool localize(const resection::SolverType & solver_type,
			const Pair & image_size,
			const cameras::IntrinsicBase * optional_intrinsics,
			const features::Regions & query_regions,
			geometry::Pose3 & pose,
			Image_Localizer_Match_Data * resection_data_ptr);
		
	private:
		std::unique_ptr<features::Image_describer> image_describer;
		std::pair <size_t, size_t> *imageSize;
		Mat3 *K;
		std::string *rootFolder;

		SfM_Data *scenePtr;

		std::unique_ptr<Regions> regions_type;
		std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();
		sfm::SfM_Localization_Single_3DTrackObservation_Database localizer;

		std::unique_ptr<features::Regions> landmark_observations_descriptors_;
		std::vector<IndexT> index_to_landmark_id_;
		std::shared_ptr<matching::Matcher_Regions_Database> matching_interface_;
	};

	bool Localizer::setupMap(LocalizationData& data)
	{
		if (data.scene.GetPoses().empty() || data.scene.GetLandmarks().empty()) {
			std::cerr << "The input scene has no 3D content." << std::endl;
			return EXIT_FAILURE;
		}

		std::string folderName = "C:/Users/saihv/Desktop/test";
		C_Progress_display progress;

		std::unique_ptr<Regions> regions_type;
		regions_type.reset(new features::AKAZE_Binary_Regions());

		regions_provider->load(data.scene, folderName, regions_type, &progress);


		if (initMatchingInterface(data.scene, *regions_provider.get())) {
			std::cerr << "Cannot initialize the SfM localizer" << std::endl;
		}
		else
			std::cout << "Initialized SFM localizer with map" << std::endl;

		scenePtr = &data.scene;

		regions_provider.reset();
	}

	bool Localizer::initMatchingInterface(SfM_Data& scene, Regions_Provider& regions_provider)
	{
		if (scene.GetPoses().empty() || scene.GetLandmarks().empty()) {
			std::cerr << std::endl << "The input SfM_Data file have not 3D content to match with." << std::endl;
			return false;
		}

		landmark_observations_descriptors_.reset(regions_provider.getRegionsType()->EmptyClone());
		for (const auto & landmark : scene.GetLandmarks())
		{
			for (const auto & observation : landmark.second.obs)
			{
				if (observation.second.id_feat != UndefinedIndexT)
				{
					// copy the feature/descriptor to landmark_observations_descriptors
					const std::shared_ptr<features::Regions> view_regions = regions_provider.get(observation.first);
					view_regions->CopyRegion(observation.second.id_feat, landmark_observations_descriptors_.get());
					index_to_landmark_id_.push_back(landmark.first);
				}
			}
		}
		std::cout << "Init retrieval database ... " << std::endl;
		matching_interface_.reset(new matching::Matcher_Regions_Database(matching::BRUTE_FORCE_HAMMING, *landmark_observations_descriptors_));
		std::cout << "Retrieval database initialized with:\n"
			<< "#landmarks: " << scene.GetLandmarks().size() << "\n"
			<< "#descriptors: " << landmark_observations_descriptors_->RegionCount() << std::endl;

		return true;
	}

	bool Localizer::localize(const resection::SolverType & solver_type,
		const Pair & image_size,
		const cameras::IntrinsicBase * optional_intrinsics,
		const features::Regions & query_regions,
		geometry::Pose3 & pose,
		Image_Localizer_Match_Data * resection_data_ptr)
	{
		if (scenePtr == nullptr || matching_interface_ == nullptr)
		{
			return false;
		}

		matching::IndMatches vec_putative_matches;
		if (!this->matching_interface_->Match(0.8, query_regions, vec_putative_matches))
		{
			return false;
		}

		std::cout << "#3D2d putative correspondences: " << vec_putative_matches.size() << std::endl;
		// Init the 3D-2d correspondences array
		Image_Localizer_Match_Data resection_data;
		if (resection_data_ptr)
		{
			resection_data.error_max = resection_data_ptr->error_max;
		}
		resection_data.pt3D.resize(3, vec_putative_matches.size());
		resection_data.pt2D.resize(2, vec_putative_matches.size());
		Mat2X pt2D_original(2, vec_putative_matches.size());
		for (size_t i = 0; i < vec_putative_matches.size(); ++i)
		{
			resection_data.pt3D.col(i) = scenePtr->GetLandmarks().at(index_to_landmark_id_[vec_putative_matches[i].i_]).X;
			resection_data.pt2D.col(i) = query_regions.GetRegionPosition(vec_putative_matches[i].j_);
			pt2D_original.col(i) = resection_data.pt2D.col(i);
			// Handle image distortion if intrinsic is known (to ease the resection)
			if (optional_intrinsics && optional_intrinsics->have_disto())
			{
				resection_data.pt2D.col(i) = optional_intrinsics->get_ud_pixel(resection_data.pt2D.col(i));
			}
		}

		const bool bResection = SfM_Localizer::Localize(
			solver_type, image_size, optional_intrinsics, resection_data, pose);

		resection_data.pt2D = std::move(pt2D_original); // restore original image domain points

		if (resection_data_ptr)
			(*resection_data_ptr) = std::move(resection_data);

		return bResection;
	}
	

	bool Localizer::localizeImage(std::string& imageName, geometry::Pose3& pose)
	{
		using namespace openMVG::features;

		image::Image<unsigned char> imageGray;

		if (!ReadImage(imageName.c_str(), &imageGray)) {
			std::cout << "Unable to read image from the given path." << std::endl;
		}
		image_describer->Describe(imageGray, regionsCurrent);
		std::cout << "#regions detected in query image: " << regionsCurrent->RegionCount() << std::endl;

		sfm::Image_Localizer_Match_Data matching_data;
		matching_data.error_max = 0.2;

		bool bSuccessfulLocalization = false;

		const openMVG::cameras::Pinhole_Intrinsic
			camL(imageSize->first, imageSize->second, (*K)(0, 0), (*K)(0, 2), (*K)(1, 2));

		// Try to localize the image in the database thanks to its regions
		if (!this->localize(
			resection::SolverType::P3P_KE_CVPR17,
			{ imageGray.Width(), imageGray.Height() },
			&camL,
			*(regionsCurrent.get()),
			pose,
			&matching_data)) {
			std::cout << "Localization unsuccessful" << std::endl;
			return EXIT_FAILURE;
		}
		else {
			std::cout << "Localization successful" << std::endl;
			return EXIT_SUCCESS;
		}
	}
}
