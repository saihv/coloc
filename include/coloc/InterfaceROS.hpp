#pragma once

#ifdef USE_STREAM
#include "coloc/colocInterface.hpp"
#include "coloc/FeatureDetector.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace coloc
{
	class ROSInterface : public Interface
	{
	public:
		void processImageSingle(const sensor_msgs::ImageConstPtr& img)
		{
			cv_bridge::CvImagePtr imagePtr;
			imagePtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
			std::cout << "Detecting features for ";
			detector.detectFeaturesTopic(0, data.GPUregions, imagePtr);
		}

		void processImagePair(const sensor_msgs::ImageConstPtr& img1, const sensor_msgs::ImageConstPtr& img2)
		{
			cv_bridge::CvImagePtr imagePtr1, imagePtr2;
			imagePtr1 = cv_bridge::toCvCopy(img1, sensor_msgs::image_encodings::MONO8);
			std::cout << "Left camera: ";
			detector.detectFeaturesTopic(0, data.GPUregions, imagePtr1);
			matcher.setTrainingImage(detector.kps, detector.desc);
			//cv::drawKeypoints(imagePtr1->image, detector.converted_kps, image_with_kps_L, cv::Scalar::all(-1.0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			//kpsL = detector.converted_kps;

			imagePtr2 = cv_bridge::toCvCopy(img2, sensor_msgs::image_encodings::MONO8);
			std::cout << "Right camera: ";
			detector.detectFeaturesTopic(1, data.GPUregions, imagePtr2);
			matcher.setQueryImage(detector.kps, detector.desc);
			//cv::drawKeypoints(imagePtr2->image, detector.converted_kps, image_with_kps_R, cv::Scalar::all(-1.0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			//kpsR = detector.converted_kps;
			detector.receivedImg = true;
		}
	};
}
#endif
