#include "openMVG/image/image.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/features/features.hpp"
#include "nonFree/sift/SIFT_describer.hpp"
#include <cereal/archives/json.hpp>
#include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/progress/progress.hpp"

#include <cstdlib>
#include <fstream>

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::features;
using namespace openMVG::sfm;
using namespace std;

features::EDESCRIBER_PRESET stringToEnum(const std::string & sPreset)
{
	features::EDESCRIBER_PRESET preset;
	if (sPreset == "NORMAL")
		preset = features::NORMAL_PRESET;
	else if (sPreset == "HIGH")
		preset = features::HIGH_PRESET;
	else if (sPreset == "ULTRA")
		preset = features::ULTRA_PRESET;
	else
		preset = features::EDESCRIBER_PRESET(-1);
	return preset;
}

/// - Compute view image description (feature & descriptor extraction)
/// - Export computed data
int main(int argc, char **argv)
{
	CmdLine cmd;

	std::string sSfM_Data_Filename;
	std::string sOutDir = "";
	bool bUpRight = false;
	std::string sImage_Describer_Method = "SIFT";
	bool bForce = false;
	std::string sFeaturePreset = "";
#ifdef OPENMVG_USE_OPENMP
	int iNumThreads = 0;
#endif

	cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
	cmd.add(make_option('o', sOutDir, "outdir"));
	cmd.add(make_option('m', sImage_Describer_Method, "describerMethod"));
	cmd.add(make_option('u', bUpRight, "upright"));
	cmd.add(make_option('f', bForce, "force"));
	cmd.add(make_option('p', sFeaturePreset, "describerPreset"));

#ifdef OPENMVG_USE_OPENMP
	cmd.add(make_option('n', iNumThreads, "numThreads"));
#endif

	try {
		if (argc == 1) throw std::string("Invalid command line parameter.");
		cmd.process(argc, argv);
	}
	catch (const std::string& s) {
		std::cerr << "Usage: " << argv[0] << '\n'
			<< "[-i|--input_file] a SfM_Data file \n"
			<< "[-o|--outdir path] \n"
			<< "\n[Optional]\n"
			<< "[-f|--force] Force to recompute data\n"
			<< "[-m|--describerMethod]\n"
			<< "  (method to use to describe an image):\n"
			<< "   SIFT (default),\n"
			<< "   CSIFT: CUDA accelerated SIFT, \n"
			<< "   SIFT_ANATOMY,\n"
			<< "   AKAZE_FLOAT: AKAZE with floating point descriptors,\n"
			<< "   AKAZE_MLDB:  AKAZE with binary descriptors\n"
			<< "[-u|--upright] Use Upright feature 0 or 1\n"
			<< "[-p|--describerPreset]\n"
			<< "  (used to control the Image_describer configuration):\n"
			<< "   NORMAL (default),\n"
			<< "   HIGH,\n"
			<< "   ULTRA: !!Can take long time!!\n"
#ifdef OPENMVG_USE_OPENMP
			<< "[-n|--numThreads] number of parallel computations\n"
#endif
			<< std::endl;

		std::cerr << s << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << " You called : " << std::endl
		<< argv[0] << std::endl
		<< "--input_file " << sSfM_Data_Filename << std::endl
		<< "--outdir " << sOutDir << std::endl
		<< "--describerMethod " << sImage_Describer_Method << std::endl
		<< "--upright " << bUpRight << std::endl
		<< "--describerPreset " << (sFeaturePreset.empty() ? "NORMAL" : sFeaturePreset) << std::endl
		<< "--force " << bForce << std::endl
#ifdef OPENMVG_USE_OPENMP
		<< "--numThreads " << iNumThreads << std::endl
#endif
		<< std::endl;
	
	if (sOutDir.empty()) {
		std::cerr << "\nIt is an invalid output directory" << std::endl;
		return EXIT_FAILURE;
	}

	if (!stlplus::folder_exists(sOutDir))
	{
		if (!stlplus::folder_create(sOutDir))
		{
			std::cerr << "Cannot create output directory" << std::endl;
			return EXIT_FAILURE;
		}
	}

	SfM_Data sfm_data;
	if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS))) {
		std::cerr << std::endl	<< "The input file \"" << sSfM_Data_Filename << "\" cannot be read" << std::endl;
		return false;
	}

	using namespace openMVG::features;
	std::unique_ptr<Image_describer> image_describer;
	const std::string sImage_describer = stlplus::create_filespec(sOutDir, "image_describer", "json");

	if (!bForce && stlplus::is_file(sImage_describer))
	{
		std::ifstream stream(sImage_describer.c_str());
		if (!stream.is_open())
			return false;

		try	{
			cereal::JSONInputArchive archive(stream);
			archive(cereal::make_nvp("image_describer", image_describer));
		}
		catch (const cereal::Exception & e)	{
			std::cerr << e.what() << std::endl	<< "Cannot dynamically allocate the Image_describer interface." << std::endl;
			return EXIT_FAILURE;
		}
	}
	else
	{
		image_describer.reset(new SIFT_Image_describer(SIFT_Image_describer::Params(), !bUpRight));
		
		{
			std::ofstream stream(sImage_describer.c_str());
			if (!stream.is_open())
				return false;

			cereal::JSONOutputArchive archive(stream);
			archive(cereal::make_nvp("image_describer", image_describer));
			std::unique_ptr<Regions> regionsType;
			image_describer->Allocate(regionsType);
			archive(cereal::make_nvp("regions_type", regionsType));
		}
	}

	{
		system::Timer timer;
		Image<unsigned char> imageGray, globalMask;

		C_Progress_display my_progress_bar(sfm_data.GetViews().size(), std::cout, "\n- EXTRACT FEATURES -\n");

#ifdef OPENMVG_USE_OPENMP
		const unsigned int nb_max_thread = omp_get_max_threads();

		if (iNumThreads > 0) {
			omp_set_num_threads(iNumThreads);
		}
		else {
			omp_set_num_threads(nb_max_thread);
		}

#pragma omp parallel for schedule(dynamic) if(iNumThreads > 0) private(imageGray)
#endif

		for (int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i)
		{
			Views::const_iterator iterViews = sfm_data.views.begin();
			std::advance(iterViews, i);
			const View * view = iterViews->second.get();
			const std::string
				sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path),
				sFeat = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "feat"),
				sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "desc");

			if (bForce || !stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc))	{
				if (!ReadImage(sView_filename.c_str(), &imageGray))
					continue;

				Image<unsigned char> * mask = nullptr; 
				std::unique_ptr<Regions> regions;
				image_describer->Describe(imageGray, regions, mask);
				image_describer->Save(regions.get(), sFeat, sDesc);

			}
#ifdef OPENMVG_USE_OPENMP
#pragma omp critical
#endif
			++my_progress_bar;
		}
		std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
	}
	return EXIT_SUCCESS;
}
