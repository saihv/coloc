#pragma once

#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/akaze/mldb_descriptor.hpp"

using namespace openMVG::features;

float GetfDescFactor()
{
	return 11.f*sqrtf(2.f);
}

std::unique_ptr<AKAZE_Image_describer_MLDB::Regions_type>
describe_AKAZE
(
	const image::Image<unsigned char>& image,
	const image::Image<unsigned char>* mask = nullptr
)
{
	auto regions = std::unique_ptr<AKAZE_Image_describer_MLDB::Regions_type>(new AKAZE_Image_describer_MLDB::Regions_type);

	if (image.size() == 0)
		return regions;

	AKAZE_Image_describer::Params params;

	params.options_.fDesc_factor = GetfDescFactor();

	AKAZE akaze(image, params.options_);
	akaze.Compute_AKAZEScaleSpace();
	std::vector<AKAZEKeypoint> kpts;
	kpts.reserve(5000);
	akaze.Feature_Detection(kpts);
	akaze.Do_Subpixel_Refinement(kpts);

	// Feature masking (remove keypoints if they are masked)
	kpts.erase(std::remove_if(kpts.begin(),
		kpts.end(),
		[&](const AKAZEKeypoint & pt)
	{
		if (mask) return (*mask)(pt.y, pt.x) == 0;
		else return false;
	}),
		kpts.end());

	regions->Features().resize(kpts.size());
	regions->Descriptors().resize(kpts.size());

#ifdef OPENMVG_USE_OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < static_cast<int>(kpts.size()); ++i) {
		AKAZEKeypoint ptAkaze = kpts[i];

		const TEvolution& cur_slice = akaze.getSlices()[ptAkaze.class_id];

		akaze.Compute_Main_Orientation(ptAkaze, cur_slice.Lx, cur_slice.Ly);

		regions->Features()[i] =
			SIOPointFeature(ptAkaze.x, ptAkaze.y, ptAkaze.size, ptAkaze.angle);

		// Compute MLDB descriptor
		Descriptor<bool, 486> desc;
		ComputeMLDBDescriptor(cur_slice.cur, cur_slice.Lx, cur_slice.Ly,
			ptAkaze.octave, regions->Features()[i], desc);
		// convert the bool vector to the binary unsigned char array
		unsigned char* ptr = reinterpret_cast<unsigned char*>(&regions->Descriptors()[i]);
		memset(ptr, 0, regions->DescriptorLength() * sizeof(unsigned char));
		// For each byte
		for (int j = 0; j < std::ceil(486. / 8.); ++j, ++ptr)
		{
			// set the corresponding 8bits to the good values
			for (int iBit = 0; iBit < 8 && j * 8 + iBit < 486; ++iBit)
			{
				*ptr |= desc[j * 8 + iBit] << iBit;
			}
		}
	}
	return regions;
}

