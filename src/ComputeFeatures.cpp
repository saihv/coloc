// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// The <cereal/archives> headers are special and must be included first.
#include <cereal/archives/json.hpp>

#include "openMVG/features/image_describer_akaze_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/regions_factory_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/system/timer.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress_display.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "nonFree/sift/SIFT_describer_io.hpp"
#include <cereal/details/helpers.hpp>

#include <atomic>
#include <cstdlib>
#include <fstream>
#include <string>

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

using namespace std;
using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::features;

namespace coloc
{
	class detector
	{
	public:
		detector(std::string& method);
		std::unique_ptr<openMVG::features::Regions> detect(std::string& imageName, std::string& method);
		int drawFeaturePoints(std::string& imageName, PointFeatures points);

	private:
		std::unique_ptr<Image_describer> image_describer;
	};

	detector::detector(std::string& method)
	{
		if (method == "SIFT")
			image_describer.reset(new SIFT_Image_describer(SIFT_Image_describer::Params(), true));
		else if (method == "AKAZE")
			image_describer = AKAZE_Image_describer::create(AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MLDB), true);
	}

	std::unique_ptr<openMVG::features::Regions> detector::detect(std::string& imageName, std::string& method)
	{		
		Image<unsigned char> imageGray;

		if (!ReadImage(imageName.c_str(), &imageGray)) {
			cout << "Unable to read image from the given path." << endl;
		}
		auto regions = image_describer->Describe(imageGray, nullptr);

		return regions;
	}
}