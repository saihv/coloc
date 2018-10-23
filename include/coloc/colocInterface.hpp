#pragma once

#include "coloc/FeatureDetector.hpp"
#include "coloc/FeatureMatcher.hpp"

#include "coloc/CPUDetector.hpp"
#include "coloc/GPUDetector.hpp"

#include "coloc/colocParams.hpp"
#include "coloc/colocData.hpp"
#include "coloc/Reconstructor.hpp"


#ifdef USE_ROS
#endif

namespace coloc
{
	class Interface
	{
	public:
		Interface(DetectorOptions &opts_, colocParams &params_, colocData &data_) : detector(opts_)
		{
			this->params = &params_;
			this->data = &data_;
		}

		unsigned int imageNumber = 0;

		virtual void processImageSingle(int &id) = 0;
		virtual void processImages() = 0;

	protected:
		DetectorOptions *opts;
		colocParams *params;
		colocData *data;
		FeatureDetector <bool, CPUDetector> detector;
	};

}