#include "stdafx.h"
#include "localizationData.hpp"
#include "featureDetector.hpp"
#include "featureMatcher.hpp"
#include "robustMatcher.hpp"

int main()
{
	coloc::LocalizationData data;
	std::string detectorType = "AKAZE";
	coloc::FeatureExtractor detector(detectorType);
	coloc::FeatureMatcher matcher(detectorType);
	coloc::RobustMatcher robustMatcher(640, 480);
	unsigned int numDrones = 2;

	std::string filename[2];
	filename[0] = "C:/Users/saihv/Desktop/test/img__Quad0_0.png";
	filename[1] = "C:/Users/saihv/Desktop/test/img__Quad1_0.png";

	for (unsigned int i = 0; i < numDrones; ++i)
		detector.detectFeatures(i, data.regions, filename[i]);

	matcher.computeMatches(data.regions, data.putativeMatches);
	robustMatcher.filterMatches(data.regions, data.putativeMatches, data.geometricMatches);

	return 0;
}

