//
// Created by sai on 7/8/18.
//

#pragma once

#include "coloc/colocData.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::features;
using namespace openMVG::matching;
using namespace openMVG::sfm;

namespace coloc
{
    class colocParams {
    public:
        std::string imageFolder;
        std::pair <int, int> imageSize;
		std::vector <Mat3> K;
        std::vector <Vec3> dist;
        char model;

		DetectorOptions detectorOptions;
		MatcherOptions matcherOptions;

        colocParams(const std::vector <Mat3> &_K,
                    const std::vector <Vec3> &_dist, const char &_model, const std::pair<size_t, size_t> &_imageSize,
                    const std::string &_imageFolder, DetectorOptions _detectorOptions, MatcherOptions _matcherOptions) :

                K(_K), dist(_dist), model(_model),
                imageSize(_imageSize), imageFolder(_imageFolder), detectorOptions(_detectorOptions), matcherOptions(_matcherOptions) {};
    };
}