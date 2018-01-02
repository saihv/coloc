#include "stdafx.h"
#include "localizationData.hpp"
#include "featureDetector.hpp"
#include "featureMatcher.hpp"
#include "robustMatcher.hpp"
#include "mapBuilder.hpp"

int main()
{
	coloc::LocalizationData data;
	std::string detectorType = "AKAZE";
	coloc::FeatureExtractor detector(detectorType);
	coloc::FeatureMatcher matcher(detectorType);
	coloc::RobustMatcher robustMatcher(640, 480);
	coloc::Reconstructor reconstructor(640, 480);

	unsigned int numDrones = 3;

	std::string filename[3];
	filename[0] = "C:/Users/saihv/Desktop/test/img__Quad0_0.png";
	filename[1] = "C:/Users/saihv/Desktop/test/img__Quad1_0.png";
	filename[2] = "C:/Users/saihv/Desktop/test/img__Quad2_0.png";

	for (unsigned int i = 0; i < numDrones; ++i)
		detector.detectFeatures(i, data.regions, filename[i]);

	matcher.computeMatches(data.regions, data.putativeMatches);
	robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches, data.relativePoses);
	reconstructor.reconstructScene(data.regions, data);

	return 0;
}

