#include "openMVG/image/image.hpp"
#include "openMVG/stl/split.hpp"
#include "openMVG/geometry/pose3.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::image;
using namespace openMVG::sfm;
using namespace openMVG::geometry;


bool checkIntrinsicStringValidity(const std::string & Kmatrix, double & focal, double & ppx, double & ppy, const std::string & Dmatrix, double & d1, double & d2, double & d3, double & d4, double & d5)
{
    std::vector<std::string> vec_str;
    stl::split(Kmatrix, ';', vec_str);
    if (vec_str.size() != 9)
    {
        std::cerr << "\n Missing ';' in intrinsics matrix" << std::endl;
        return false;
    }

    for (size_t i = 0; i < vec_str.size(); ++i)
    {
        double readvalue = 0.0;
        std::stringstream ss;
        ss.str(vec_str[i]);
        if (! (ss >> readvalue) )
        {
            std::cerr << "\n Invalid character detected in K matrix" << std::endl;
            return false;
        }
        if (i==0) focal = readvalue;
        if (i==2) ppx = readvalue;
        if (i==5) ppy = readvalue;
    }

    std::vector<std::string> vec_str2;
    stl::split(Dmatrix, ';', vec_str2);
    if (vec_str2.size() != 5)
    {
        std::cerr << "\n Missing ';' in distortion matrix" << std::endl;
        return false;
    }

    for (size_t i = 0; i < vec_str2.size(); ++i)
    {
        double readvalue = 0.0;
        std::stringstream ss;
        ss.str(vec_str2[i]);
        if (! (ss >> readvalue) )
        {
            std::cerr << "\n Invalid character detected in distortion matrix" << std::endl;
            return false;
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    CmdLine cmd;

    std::string sImageDir, sfileDatabase = "", sOutputDir = "", sKmatrix, sDistmatrix;

    int i_User_camera_model = 5;
    bool b_Group_camera_model = true;
    double focal_pixels = -1.0;
    double baseline = 1.0;

    cmd.add( make_option('i', sImageDir, "imageDirectory") );
    cmd.add( make_option('o', sOutputDir, "outputDirectory") );
    cmd.add( make_option('f', focal_pixels, "focal") );
    cmd.add( make_option('k', sKmatrix, "intrinsics") );
    cmd.add( make_option('d', sDistmatrix, "distortion") );
    cmd.add( make_option('b', baseline, "baseline") );
    cmd.add( make_option('c', i_User_camera_model, "camera_model") );
    cmd.add( make_option('g', b_Group_camera_model, "group_camera_model") );

    try
    {
        if (argc == 1) throw std::string("Invalid command line parameter.");
        cmd.process(argc, argv);
    }
    catch(const std::string& s)
    {
        std::cerr << "Usage: " << argv[0] << '\n'
        << "[-i|--imageDirectory]\n"
        << "[-o|--outputDirectory]\n"
        << "[-f|--focal] (pixels)\n"
        << "[-k|--intrinsics] Kmatrix: \"f;0;ppx;0;f;ppy;0;0;1\"\n"
        << "[-c|--camera_model] Camera model type:\n"
        << "\t 1: Pinhole\n"
        << "\t 2: Pinhole radial 1\n"
        << "\t 3: Pinhole radial 3 (default)\n"
        << "\t 4: Pinhole brown 2\n"
        << "\t 5: Pinhole with a simple Fish-eye distortion\n"
        << "[-g|--group_camera_model]\n"
        << "\t 0-> each view have it's own camera intrinsic parameters,\n"
        << "\t 1-> (default) view can share some camera intrinsic parameters\n"
        << std::endl;

        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Provided data: " << std::endl
    << "Camera intrinsics: " << sKmatrix << std::endl
    << "Camera distortion: " << sDistmatrix << std::endl;

    double width = -1, height = -1, focal = -1, ppx = -1,  ppy = -1, d1 = 0.0, d2 = 0.0, d3 = 0.0, d4 = 0.0, d5 = 0.0;

    const EINTRINSIC e_User_camera_model = EINTRINSIC(i_User_camera_model);

    if ( !stlplus::folder_exists( sImageDir ) )
    {
        std::cerr << "\nThe input directory doesn't exist" << std::endl;
        return EXIT_FAILURE;
    }

    if (sOutputDir.empty())
    {
        std::cerr << "\nInvalid output directory" << std::endl;
        return EXIT_FAILURE;
    }

    if ( !stlplus::folder_exists( sOutputDir ) )
    {
        if ( !stlplus::folder_create( sOutputDir ))
        {
            std::cerr << "\nCannot create output directory" << std::endl;
            return EXIT_FAILURE;
        }
    }

    if (sKmatrix.size() > 0 &&
    !checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy, sDistmatrix, d1, d2, d3, d4, d5) )
    {
        std::cerr << "\nInvalid K/distortion matrix input" << std::endl;
        return EXIT_FAILURE;
    }

    if (sKmatrix.size() > 0 && focal_pixels != -1.0)
    {
        std::cerr << "\nCannot combine -f and -k options" << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<std::string> vec_image = stlplus::folder_files( sImageDir );
    std::sort(vec_image.begin(), vec_image.end());

    // Configure an empty scene with Views and their corresponding cameras
    SfM_Data SfM_Data;
    SfM_Data.s_root_path = sImageDir; // Setup main image root_path
    Views & views = SfM_Data.views;
    Intrinsics & intrinsics = SfM_Data.intrinsics;
    Poses & poses = SfM_Data.poses;

    std::ostringstream error_report_stream;
    for ( std::vector<std::string>::const_iterator iter_image = vec_image.begin();  iter_image != vec_image.end(); ++iter_image)
    {
        // Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
        width = height = ppx = ppy = focal = -1.0;

        const std::string sImageFilename = stlplus::create_filespec( sImageDir, *iter_image );
        const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

        // Test if the image format is supported:
        if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown)
        {
            error_report_stream << sImFilenamePart << ": Unkown image file format." << "\n";
            continue; // image cannot be opened
        }

        ImageHeader imgHeader;
        if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
            continue; // image cannot be read

        width = imgHeader.width;
        height = imgHeader.height;
        ppx = width / 2.0;
        ppy = height / 2.0;

        // Consider the case where the focal is provided manually
        if (focal_pixels != -1)
        {
            if (sKmatrix.size() > 0 && sDistmatrix.size() > 0) // Known user calibration K matrix
            {
                if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy, sDistmatrix, d1, d2, d3, d4, d5))
                focal = -1.0;
            }
            else // User provided focal length value
            {
                focal = focal_pixels;
            }
        }

        // Build intrinsic parameter related to the view
        std::shared_ptr<IntrinsicBase> intrinsic (NULL);

        if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0)
            intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>(width, height, focal, ppx, ppy, d1, d2, d3, d4, d5);

        View v(*iter_image, views.size(), views.size(), views.size(), width, height);

        // Add intrinsic related to the image (if any)
        if (intrinsic == NULL)
            v.id_intrinsic = UndefinedIndexT;
        else
            intrinsics[v.id_intrinsic] = intrinsic;

        poses[v.id_pose] = Pose3(Mat3::Identity(), Vec3(0,0,0));

        if(v.id_pose == 1)
            poses[v.id_pose] = Pose3(Mat3::Identity(), Vec3(baseline,0,0));

        views[v.id_view] = std::make_shared<View>(v);
    }

    // Display saved warning & error messages if any.
    if (!error_report_stream.str().empty())
    {
        std::cerr << "\nWarning & Error messages:" << std::endl << error_report_stream.str() << std::endl;
    }

    // Group camera that share common properties if desired (leads to more faster & stable BA).
    if (b_Group_camera_model)
        GroupSharedIntrinsics(SfM_Data);

    // Store SfM_Data views & intrinsic data
    if (!Save(SfM_Data, stlplus::create_filespec( sOutputDir, "SfM_Data.json" ).c_str(), ESfM_Data(VIEWS|EXTRINSICS|INTRINSICS)))
        return EXIT_FAILURE;

    std::cout << std::endl
    << "Image listing:\n"
    << "Files found: " << vec_image.size() << "\n"
    << "Files enumerated: " << SfM_Data.GetViews().size() << std::endl;

    return EXIT_SUCCESS;
}
