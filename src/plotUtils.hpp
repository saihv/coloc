#pragma once

#include "localizationUtils.hpp"
#include "localizationData.hpp"
#include "localizationParams.hpp"

#include "Python.h"

namespace coloc
{
	class Plotter
	{
	public:
		Plotter()
		{
			Py_Initialize();
			PyRun_SimpleString("import matplotlib.pyplot as plt");
			PyRun_SimpleString("from mpl_toolkits.mplot3d import Axes3D");
			PyRun_SimpleString("fig = plt.figure()");
			PyRun_SimpleString("axes = fig.add_subplot(111, projection = '3d')");
		}
		bool plotScene(Scene& scene);

	private:
		bool plotLocation(Vec3& location);
		bool plotOrientation(Mat3& rmat);
		bool plotMap(Scene& scene);
	};

	bool Plotter::plotLocation(Vec3& location)
	{
		std::string command = "axes.scatter(";
		for (unsigned int i = 0; i < 3; i++) {
			command += std::to_string(location[i]);
			command += ", ";
		}
		command += "s=20)";

		PyRun_SimpleString(command.c_str());
	}

	bool Plotter::plotMap(Scene& scene)
	{
		Landmarks map = scene.GetLandmarks();
		std::vector <double> x, y, z;

		for (auto landmark : map) {
			Vec3 loc = landmark.second.X;
			x.push_back(loc[0]);
			y.push_back(loc[1]);
			z.push_back(loc[2]);
		}

		std::string command = "axes.scatter([";

		for (int i = 0; i < x.size() - 1; i++) {
			command += std::to_string(x[i]);
			command += ", ";
		}
		command += std::to_string(x.back());
		command += "], [";
		for (int i = 0; i < z.size() - 1; i++) {
			command += std::to_string(z[i]);
			command += ", ";
		}
		command += std::to_string(z.back());
		command += "], [";
		for (int i = 0; i < y.size() - 1; i++) {
			command += std::to_string(-1*y[i]);
			command += ", ";
		}
		command += std::to_string(y.back());
		command += "],c='limegreen')";

		PyRun_SimpleString(command.c_str());
	}

	bool Plotter::plotScene(Scene& scene)
	{
		for (auto pose : scene.poses) {
			Pose3 currentPose = pose.second;

			Mat3 R = currentPose.rotation();
			Vec3 t = currentPose.translation();

			plotLocation(t);
		}

		plotMap(scene);
		PyRun_SimpleString("plt.show()");
		return true;
	}

}
