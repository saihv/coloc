#include "coloc/colocInterface.hpp"
#include "coloc/FeatureDetector.hpp"

namespace coloc
{
	class DiskInterface : public Interface
	{
	public:
		using Interface::Interface;

		void processImageSingle(int &id) override
		{
			std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);  //std::to_string(imageNumber); //std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);
			data->filenames[id] = params->imageFolder + "img__Quad" + std::to_string(id) + "_" + number + ".png";  //"image (" + number + ").png";
			detector.detectFeaturesFile(id, data->regions, data->filenames[id]);
		}

		void processImages() override
		{
			std::vector <std::string> filename;
			std::string number = std::string(4 - std::to_string(imageNumber).length(), '0') + std::to_string(imageNumber);
			auto start = std::chrono::system_clock::now();
			for (unsigned int i = 0; i < data->numDrones; ++i) {
				data->filenames[i] = params->imageFolder + "img__Quad" + std::to_string(i) + "_" + number + ".png";
				std::cout << data->filenames[i] << std::endl;

				detector.detectFeaturesFile(i, data->regions, data->filenames[i]);
				//detector.saveFeatureData(i, data.regions, filename[i]);

				data->scene.views[i].reset(new View(data->filenames[i], i, 0, i, params->imageSize.first, params->imageSize.second));
			}
		}
	};
}