//
// Created by sai on 7/8/18.
//
#pragma once

#include "openMVG.h"
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
    typedef std::vector<std::array<double, 6 * 6>> Cov6;
    typedef std::map<IndexT, std::unique_ptr<Regions> > FeatureMap;
    // typedef std::map<IndexT, std::unique_ptr<AKAZE_Binary_Regions> > GPUFeatureMap;
    typedef std::map<Pair, RelativePose_Info> InterPoseMap;
    typedef SfM_Data Scene;
    typedef cameras::IntrinsicBase Camera;

	struct DetectorOptions {
		float scale_factor;
		uint8_t scale_levels;
		unsigned int width;
		unsigned int height;
		unsigned int maxkp;
		uint8_t thresh;
	};

	struct MatcherOptions {
		float distRatio;
		uint8_t thresh;
		unsigned int maxkp;
	};


    class colocData {
    public:
        FeatureMap regions;
        PairWiseMatches putativeMatches, geometricMatches;
        InterPoseMap relativePoses;
        std::map<Pair, double> overlap;
        Scene scene, tempScene;
        std::unique_ptr<features::Regions> mapRegions;
		std::unique_ptr<features::Regions> interMapRegions;
        std::vector <IndexT> mapRegionIdx;
		std::vector <IndexT> interMapRegionIdx;
		Camera* cam;
		unsigned int numDrones;
		unsigned int keyframeIdx;
		std::vector <std::string> filenames;
		std::vector <std::string> keyframeNames;

		colocData & operator = (colocData &data)
		{
			// regions = data.regions;
			this->putativeMatches = data.putativeMatches;
			this->geometricMatches = data.geometricMatches;
			this->relativePoses = data.relativePoses;
			this->overlap = data.overlap;
			this->scene = data.scene;

			//std::unique_ptr<Regions> regions = std::make_unique<Regions>();
			//for (auto const & x : data.regions)
			//	this->regions.insert({ x.first, std::make_unique<Regions>(*x.second) });
			//this->regions.emplace_hint(this->regions.end(), x.first, std::make_unique<Regions>(*x.second));

			this->mapRegions = std::move(data.mapRegions);
			this->mapRegionIdx = data.mapRegionIdx;
			return *this;
		}

		bool setCameraIntrinsics(Mat3 &K, Vec3 &dist, std::pair<int, int> &imageSize)
		{
			const openMVG::cameras::Pinhole_Intrinsic_Radial_K3 cam(imageSize.first, imageSize.second, (K)(0, 0), (K)(0, 2), (K)(1, 2), dist[0], dist[1], dist[2]);
		}

        bool setupMapDatabase(bool inter)
        {
			mapRegions.reset(new AKAZE_Binary_Regions);
			Scene *map;
			std::vector <IndexT> *indexes;
			std::unique_ptr<features::Regions> *features;

			if (inter) {
				map = &this->tempScene;
				indexes = &this->interMapRegionIdx;
				features = &this->interMapRegions;
			}
			else {
				map = &this->scene;
				indexes = &this->mapRegionIdx;
				features = &this->mapRegions;
			}

			for (const auto &landmark : map->GetLandmarks()) {
				const auto &observation = landmark.second.obs.begin();
				//for (const auto &observation : landmark.second.obs) {
					if (observation->second.id_feat != UndefinedIndexT) {
						regions.at(observation->first)->CopyRegion(observation->second.id_feat, features->get());
						mapRegionIdx.push_back(landmark.first);
					}
				//}
			}
			return EXIT_SUCCESS;
        }
    };
}