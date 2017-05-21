/*******************************************************************
*   main.cpp
*   CUDAK2NN
*
*	Author: Kareem Omar
*	kareem.omar@uah.edu
*	https://github.com/komrad36
*
*	Last updated Oct 12, 2016
*******************************************************************/
//
// Fastest GPU implementation of a brute-force
// matcher for 512-bit binary descriptors
// in 2NN mode, i.e., a match is returned if the best
// match between a query vector and a training vector
// is more than a certain threshold number of bits
// better than the second-best match.
//
// Yes, that means the DIFFERENCE in popcounts is used
// for thresholding, NOT the ratio. This is the CORRECT
// approach for binary descriptors.
//
// This laboriously crafted kernel is EXTREMELY fast.
// 63 BILLION comparisons per second on a stock GTX1080,
// enough to match nearly 46,000 descriptors per frame at 30 fps (!)
//
// A key insight responsible for much of the performance of
// this insanely fast CUDA kernel is due to
// Christopher Parker (https://github.com/csp256), to whom
// I am extremely grateful.
//
// CUDA CC 3.0 or higher is required.
//
// All functionality is contained in the files CUDAK2NN.h
// and CUDAK2NN.cu. 'main.cpp' is simply a sample test harness
// with example usage and performance testing.
//

#include "CUDAK2NN.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "KORAL.h"
using namespace std::chrono;

struct Match {
	int q, t;
	Match() {}
	Match(const int _q, const int _t) : q(_q), t(_t) {}
};

int main() {
	// ------------- Configuration ------------
	constexpr int warmups = 100;
	constexpr int runs = 300;
	constexpr int size = 10000;
	constexpr int threshold = 5;

	constexpr uint8_t KFAST_thresh = 60;
	constexpr float scale_factor = 1.2f;
	constexpr uint8_t scale_levels = 8;
	// --------------------------------

	cv::Mat image = cv::imread("1.png", CV_LOAD_IMAGE_GRAYSCALE);
	if (!image.data) {
		std::cerr << "ERROR: failed to open image. Aborting." << std::endl;
		return EXIT_FAILURE;
	}

	KORAL koral(scale_factor, scale_levels);
	koral.go(image.data, image.cols, image.rows, KFAST_thresh);

	std::cout << "Found " << koral.kps.size() << " keypoints and " << koral.desc.size() << " descriptors" << std::endl;

	uint8_t u8[8];
	void *qvecs = malloc(64 * koral.kps.size());
	int ctr = 0;

	for (unsigned int i = 0 ; i < koral.desc.size() ; i++) {
		std::memcpy(u8, &koral.desc[i], sizeof(koral.desc[i]));
		for (unsigned int j = 0; j < 8 ; j++) {
			reinterpret_cast<uint8_t*>(qvecs)[ctr] = u8[j];
			ctr++;
		}
	}

	image = cv::imread("2.png", CV_LOAD_IMAGE_GRAYSCALE);
	if (!image.data) {
		std::cerr << "ERROR: failed to open image. Aborting." << std::endl;
		return EXIT_FAILURE;
	}

	// KORAL koral(scale_factor, scale_levels);
	koral.go(image.data, image.cols, image.rows, KFAST_thresh);

	std::cout << "Found " << koral.kps.size() << " keypoints and " << koral.desc.size() << " descriptors" << std::endl;

	void *tvecs = malloc(64 * koral.kps.size());
	ctr = 0;

	for (unsigned int i = 0 ; i < koral.desc.size() ; i++) {
		std::memcpy(u8, &koral.desc[i], sizeof(koral.desc[i]));
		for (unsigned int j = 0; j < 8 ; j++) {
			reinterpret_cast<uint8_t*>(tvecs)[ctr] = u8[j];
			ctr++;
		}
	}


	// setting cache and shared modes
	cudaDeviceSetCacheConfig(cudaFuncCachePreferL1);
	cudaDeviceSetSharedMemConfig(cudaSharedMemBankSizeEightByte);

	// allocating and transferring query vecs and binding to texture object
	void* d_qvecs;
	cudaMalloc(&d_qvecs, 64 * size);
	cudaMemcpy(d_qvecs, qvecs, 64 * size, cudaMemcpyHostToDevice);
	struct cudaResourceDesc resDesc;
	memset(&resDesc, 0, sizeof(resDesc));
	resDesc.resType = cudaResourceTypeLinear;
	resDesc.res.linear.devPtr = d_qvecs;
	resDesc.res.linear.desc.f = cudaChannelFormatKindUnsigned;
	resDesc.res.linear.desc.x = 32;
	resDesc.res.linear.desc.y = 32;
	resDesc.res.linear.sizeInBytes = 64 * size;
	struct cudaTextureDesc texDesc;
	memset(&texDesc, 0, sizeof(texDesc));
	texDesc.addressMode[0] = cudaAddressModeBorder;
	texDesc.addressMode[1] = cudaAddressModeBorder;
	texDesc.filterMode = cudaFilterModePoint;
	texDesc.readMode = cudaReadModeElementType;
	texDesc.normalizedCoords = 0;
	cudaTextureObject_t tex_q = 0;
	cudaCreateTextureObject(&tex_q, &resDesc, &texDesc, nullptr);

	// allocating and transferring training vecs as simple global
	// NOTE: always allocate 8 EXTRA AT THE END. Contents don't
	// matter but the allocation does.
	void* d_tvecs;
	cudaMalloc(&d_tvecs, 64 * (size + 8));
	cudaMemcpy(d_tvecs, tvecs, 64 * size, cudaMemcpyHostToDevice);

	// allocating space for match results
	int* d_matches;
	cudaMalloc(&d_matches, 4 * size);

	std::cout << std::endl << "Warming up..." << std::endl;
	for (int i = 0; i < warmups; ++i) CUDAK2NN(d_tvecs, size, tex_q, size, d_matches, threshold);
	std::cout << "Testing..." << std::endl;
	high_resolution_clock::time_point start = high_resolution_clock::now();
	for (int i = 0; i < runs; ++i) CUDAK2NN(d_tvecs, size, tex_q, size, d_matches, threshold);
	high_resolution_clock::time_point end = high_resolution_clock::now();
	// --------------------------------


	// transferring matches back to host
	int* h_matches = reinterpret_cast<int*>(malloc(4 * size));
	cudaMemcpy(h_matches, d_matches, 4 * size, cudaMemcpyDeviceToHost);
	cudaDeviceReset();

	std::cout << "CUDA reports " << cudaGetErrorString(cudaGetLastError()) << std::endl;

	std::vector<Match> matches;
	for (int i = 0; i < size; ++i) {
		if (h_matches[i] != -1) matches.emplace_back(i, h_matches[i]);
	}

	double sec = static_cast<double>(duration_cast<nanoseconds>(end - start).count()) * 1e-9 / static_cast<double>(runs);
	std::cout << "CUDAK2NN found " << matches.size() << " matches in " << sec * 1e3 << " ms" << std::endl;
	std::cout << "Throughput: " << static_cast<double>(size)*static_cast<double>(size) / sec * 1e-9 << " billion comparisons/second." << std::endl << std::endl;
}
