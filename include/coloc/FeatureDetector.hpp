//
// Created by sai on 7/10/18.
//

#pragma once

#ifdef USE_ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "coloc/colocData.hpp"

template <typename T, template <class> class ProcessorType>
class FeatureDetector : public ProcessorType<T> {
public:
	FeatureDetector(coloc::DetectorOptions &opts) : ProcessorType <T> (opts)
	{
	
	}

	bool detectFeaturesFile(unsigned int idx, coloc::FeatureMap &regions, std::string &imageName)
	{
		return ProcessorType<T>::detectFeaturesFile(idx, regions, imageName);
	}

#ifdef USE_STREAM
    virtual void detectFeaturesTopic(uint8_t, coloc::FeatureMap&, cv_bridge::CvImagePtr) = 0;
#endif
};