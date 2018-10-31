#pragma once

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
class FeatureMatcher : public ProcessorType<T> {
public:
	FeatureMatcher(coloc::MatcherOptions &opts) : ProcessorType <T> (opts)
	{
		
	}


	bool computeMatches(coloc::FeatureMap& regions, PairWiseMatches &putativeMatches) {
		return ProcessorType <T> ::computeMatches(regions, putativeMatches);
	}
};
