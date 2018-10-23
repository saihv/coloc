#ifdef USE_CUDA

#include <cuda.h>
#include <cuda_runtime.h>
#include "device_launch_parameters.h"

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
#include "coloc/CUDAK2NN.h"
#include "coloc/Keypoint.h"
#include <chrono>

#include "coloc/colocData.hpp"
#include "coloc/colocParams.hpp"

using namespace openMVG::matching;

#define cudaCalloc(A, B, STREAM) \
    do { \
        cudaError_t __cudaCalloc_err = cudaMalloc(A, B); \
        if (__cudaCalloc_err == cudaSuccess) cudaMemsetAsync(*A, 0, B, STREAM); \
} while (0)

namespace coloc 
{
	template <typename T>
	class GPUMatcher{
	public:
		uint64_t *d_descQ, *d_descT;
		uint maxkpNum;
		std::vector<cv::DMatch> dmatches;

	private:
		unsigned int kpTrain, kpQuery;
		const uint8_t matchThreshold;
		struct Match {
			int q, t;
			Match() {}
			Match(const int _q, const int _t) : q(_q), t(_t) {}
		};

		struct cudaResourceDesc resDesc;
		struct cudaTextureDesc texDesc;
		cudaTextureObject_t tex_q = 0;
		int* d_matches;
		cudaStream_t m_stream1, m_stream2;

		int* h_matches;

	public:
		GPUMatcher(MatcherOptions opts) : matchThreshold(opts.thresh), maxkpNum(opts.maxkp)
		{
			if (cudaStreamCreate(&m_stream1) == cudaErrorInvalidValue || cudaStreamCreate(&m_stream2) == cudaErrorInvalidValue)
				std::cerr << "Unable to create stream" << std::endl;

			cudaCalloc((void**)&d_descQ, 64 * maxkpNum, m_stream1);
			cudaCalloc((void**)&d_descT, 64 * maxkpNum, m_stream2);

			memset(&resDesc, 0, sizeof(resDesc));
			memset(&texDesc, 0, sizeof(texDesc));

			resDesc.resType = cudaResourceTypeLinear;
			resDesc.res.linear.desc.f = cudaChannelFormatKindUnsigned;
			resDesc.res.linear.desc.x = 32;
			resDesc.res.linear.desc.y = 32;

			texDesc.addressMode[0] = cudaAddressModeBorder;
			texDesc.addressMode[1] = cudaAddressModeBorder;
			texDesc.filterMode = cudaFilterModePoint;
			texDesc.readMode = cudaReadModeElementType;
			texDesc.normalizedCoords = 0;

			cudaMalloc(&d_matches, 4 * maxkpNum);
			h_matches = reinterpret_cast<int*>(malloc(4 * kpQuery));
		}

		~GPUMatcher()
		{
		}

		void freeGPUMemory()
		{
			cudaFree(d_descQ);
			cudaFree(d_descT);
			cudaFree(d_matches);
		}

		void setMapData(std::vector<Keypoint> const &kps, std::vector<uint64_t> const &desc)
		{
			kpMap = kps.size();
			cudaMemsetAsync(d_descM, 0, 64 * (kpMap + 8), m_stream1);
			cudaMemcpyAsync(d_descM, &desc[0], 64 * (kpMap + 8), cudaMemcpyHostToDevice, m_stream1);
			cudaStreamSynchronize(m_stream1);
		}

		// Allocate memory and transfer descriptors for training image
		void setTrainingImage(std::vector<Keypoint> const &kps, std::vector<uint64_t> const &desc)
		{
			kpTrain = kps.size();
			cudaMemsetAsync(d_descT, 0, 64 * (kpTrain + 8), m_stream1);
			cudaMemcpyAsync(d_descT, &desc[0], 64 * (kpTrain + 8), cudaMemcpyHostToDevice, m_stream1);
			cudaStreamSynchronize(m_stream1);
		}

		// Allocate memory and transfer descriptors for query image
		void setQueryImage(std::vector<Keypoint> const &kps, std::vector<uint64_t> const &desc)
		{
			kpQuery = kps.size();
			cudaMemsetAsync(d_descQ, 0, 64 * (kpQuery), m_stream2);
			cudaMemcpyAsync(d_descQ, &desc[0], 64 * (kpQuery), cudaMemcpyHostToDevice, m_stream2);
			cudaStreamSynchronize(m_stream2);

			resDesc.res.linear.devPtr = d_descQ;
			resDesc.res.linear.sizeInBytes = 64 * kps.size();

			cudaCreateTextureObject(&tex_q, &resDesc, &texDesc, nullptr);
		}

		T computeMatches(FeatureMap& regions, PairWiseMatches &putativeMatches) {
			int numImages = static_cast<int> (regions.size());
			Pair_Set pairs = handlePairs(numImages);

			for (const auto pairIdx : pairs) {
				computeMatchesPair(pairIdx, regions.at(pairIdx.first), regions.at(pairIdx.second), putativeMatches);
			}

			return EXIT_SUCCESS;
		}

		void computeMatchesPair(const Pair& pairIdx, std::unique_ptr<features::AKAZE_Binary_Regions>& regions1, std::unique_ptr<features::AKAZE_Binary_Regions>& regions2, PairWiseMatches &putativeMatches)
		{
			IndMatches vec_PairMatches;
			vec_PairMatches = computePairMatches(const_cast<unsigned int*>(static_cast<const unsigned int*>(regions1->DescriptorRawData())),
				const_cast<unsigned int*>(static_cast<const unsigned int*>(regions2->DescriptorRawData())),
				regions1->RegionCount(),
				regions2->RegionCount());

			if (!vec_PairMatches.empty()) {
				putativeMatches.insert({ pairIdx, std::move(vec_PairMatches) });
				// overlap.insert({ pairIdx, std::move(vec_PutativeMatches.size()) });
			}
		}

		IndMatches computePairMatches(void* h_descriptorsQuery, void* h_descriptorsTraining, int numKPQuery, int numKPTraining)
		{
			kpQuery = numKPQuery;
			kpTrain = numKPTraining;

			const size_t sizeDQuery = numKPQuery * 64; // D1 for descriptor1
			const size_t sizeDTraining = (numKPTraining + 8) * 64; // D2 for descriptor2

			cudaMemsetAsync(d_descQ, 0, sizeDQuery, m_stream1);
			cudaMemsetAsync(d_descT, 0, sizeDTraining, m_stream2);

			cudaMemcpyAsync(d_descQ, h_descriptorsQuery, sizeDQuery, cudaMemcpyHostToDevice, m_stream1);
			cudaMemcpyAsync(d_descT, h_descriptorsTraining, sizeDTraining, cudaMemcpyHostToDevice, m_stream2);

			cudaStreamSynchronize(m_stream1);
			cudaStreamSynchronize(m_stream2);

			resDesc.res.linear.devPtr = d_descQ;
			resDesc.res.linear.sizeInBytes = 64 * numKPQuery;

			cudaCreateTextureObject(&tex_q, &resDesc, &texDesc, nullptr);

			cudaMemset(d_matches, 0, static_cast<int>(kpQuery));
			auto start = high_resolution_clock::now();
			CUDAK2NN(d_descT, static_cast<int>(kpTrain), tex_q, static_cast<int>(kpQuery), d_matches, matchThreshold);
			auto end = high_resolution_clock::now();

			std::vector<int> h_matches(kpQuery);
			cudaMemcpy(&h_matches[0], d_matches, 4 * kpQuery, cudaMemcpyDeviceToHost);
			//std::vector<Match> matches;

			openMVG::matching::IndMatches matches;

			dmatches.clear();
			for (size_t i = 0; i < kpQuery; ++i) {
				if (h_matches[i] != -1) {
					matches.emplace_back(i, h_matches[i]);
					dmatches.emplace_back(h_matches[i], i, 0.0f);
				}
			}

			auto sec = static_cast<double>(duration_cast<milliseconds>(end - start).count());
			std::cout << "Computed " << matches.size() << " matches in " << sec << " ms" << std::endl;

			return matches;
		}

			// Perform brute force matching between training and query images
		IndMatches matchFeaturesPair() override
		{
			cudaMemset(d_matches, 0, static_cast<int>(kpQuery));
			auto start = std::chrono::high_resolution_clock::now();
			CUDAK2NN(d_descT, static_cast<int>(kpTrain), tex_q, static_cast<int>(kpQuery), d_matches, matchThreshold);
			auto end = std::chrono::high_resolution_clock::now();

			std::vector<int> h_matches(kpQuery);
			cudaMemcpy(&h_matches[0], d_matches, 4 * kpQuery, cudaMemcpyDeviceToHost);
			IndMatches matches;
			dmatches.clear();
			for (size_t i = 0; i < kpQuery; ++i) {
				if (h_matches[i] != -1) {
					matches.emplace_back(i, h_matches[i]);
					dmatches.emplace_back(h_matches[i], i, 0.0f);
				}
			}
			auto sec = static_cast<double>(duration_cast<std::chrono::nanoseconds>(end - start).count()) * 1e-9 / static_cast<double>(1);
			std::cout << "Computed " << matches.size() << " matches in " << sec * 1e3 << " ms" << std::endl;

			return matches;
		}

		IndMatches matchFeaturesWithMap(const int *idx, colocData &data)
		{
			vec_PairMatches = computePairMatches(const_cast<unsigned int*>(static_cast<const unsigned int*>(data.regions[idx]->DescriptorRawData())),
				const_cast<unsigned int*>(static_cast<const unsigned int*>(data.mapRegions->DescriptorRawData())),
				data.regions[idx]->RegionCount(),
				data.mapRegions->RegionCount());

			if (!vec_PairMatches.empty()) {
				putativeMatches.insert({ pairIdx, std::move(vec_PairMatches) });
				// overlap.insert({ pairIdx, std::move(vec_PutativeMatches.size()) });
			}

			kpQuery = numKPQuery;
			kpTrain = numKPTraining;

			const size_t sizeDQuery = numKPQuery * 64; // D1 for descriptor1
			const size_t sizeDTraining = (numKPTraining + 8) * 64; // D2 for descriptor2

			cudaMemsetAsync(d_descQ, 0, sizeDQuery, m_stream1);
			//cudaMemsetAsync(d_descT, 0, sizeDTraining, m_stream2);

			cudaMemcpyAsync(d_descQ, data.regions[idx]->DescriptorRawData(), sizeDQuery, cudaMemcpyHostToDevice, m_stream1);
			//cudaMemcpyAsync(d_descT, h_descriptorsTraining, sizeDTraining, cudaMemcpyHostToDevice, m_stream2);

			cudaStreamSynchronize(m_stream1);
			//cudaStreamSynchronize(m_stream2);

			resDesc.res.linear.devPtr = d_descQ;
			resDesc.res.linear.sizeInBytes = 64 * numKPQuery;

			cudaCreateTextureObject(&tex_q, &resDesc, &texDesc, nullptr);

			cudaMemset(d_matches, 0, static_cast<int>(kpQuery));
			auto start = high_resolution_clock::now();
			CUDAK2NN(d_descT, static_cast<int>(kpTrain), tex_q, static_cast<int>(kpQuery), d_matches, matchThreshold);
			auto end = high_resolution_clock::now();

			std::vector<int> h_matches(kpQuery);
			cudaMemcpy(&h_matches[0], d_matches, 4 * kpQuery, cudaMemcpyDeviceToHost);
			//std::vector<Match> matches;

			openMVG::matching::IndMatches matches;

			dmatches.clear();
			for (size_t i = 0; i < kpQuery; ++i) {
				if (h_matches[i] != -1) {
					matches.emplace_back(i, h_matches[i]);
					dmatches.emplace_back(h_matches[i], i, 0.0f);
				}
			}

			auto sec = static_cast<double>(duration_cast<milliseconds>(end - start).count());
			std::cout << "Computed " << matches.size() << " matches in " << sec << " ms" << std::endl;

			return matches;
		}
	};
}
#endif
