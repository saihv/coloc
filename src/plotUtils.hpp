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
			PyRun_SimpleString("import numpy as np");
			PyRun_SimpleString("fig = plt.figure()");
			PyRun_SimpleString("axes = fig.add_subplot(111, projection = '3d')");
		}
		bool plotScene(Scene& scene);
		bool plotPoseandCovariance(Pose3& pose, Cov6& cov);
		bool plotPose(Pose3& pose, uint8_t flag);
		void drawPlot();

	private:
		bool plotLocation(Vec3& location, std::string& color);
		bool plotOrientation(Mat3& rmat, Vec3& pos, std::string& color);
		bool plotMap(Scene& scene);

		std::string seedPoseColor = "crimson";
		std::string intraLocPoseColor = "lime";
		std::string interLocPoseColor = "crimson";
		std::string mapColor = "midnightblue";
	};

	bool Plotter::plotLocation(Vec3& location, std::string& color)
	{
		std::string command = "axes.scatter(";
		command += std::to_string(location[0]);
		command += ", ";
		command += std::to_string(location[2]);
		command += ", ";
		command += std::to_string(location[1]);
		command += ", ";
		command += "s=20, c='" + color + "')";

		PyRun_SimpleString(command.c_str());
		return true;
	}

	bool Plotter::plotOrientation(Mat3& rmat, Vec3& pos, std::string& color)
	{
		Vec3 eulerAngles = rmat.eulerAngles(2, 1, 0);

		float roll = eulerAngles[0];
		float pitch = eulerAngles[1];
		float yaw = eulerAngles[2];

		float u = -sin(roll) * cos(yaw) - (cos(roll) * sin(pitch) * sin(yaw));
		float v = sin(roll) * sin(yaw) - (cos(roll) * sin(pitch) * cos(yaw));
		float w = cos(roll) * cos(pitch);

		std::string command = "axes.quiver(";
		command += std::to_string(pos[0]);
		command += ", ";
		command += std::to_string(pos[2]);
		command += ", ";
		command += std::to_string(pos[1]);
		command += ", ";
		command += std::to_string(u);
		command += ", ";
		command += std::to_string(v);
		command += ", ";
		command += std::to_string(w);
		command += ", ";
		command += "length=5, normalize=True)";

		PyRun_SimpleString(command.c_str());
		
		return Success;
	}

	bool Plotter::plotMap(Scene& scene)
	{
		Landmarks map = scene.GetLandmarks();
		std::vector <double> x, y, z;

		for (auto landmark : map) {
			Vec3 loc = landmark.second.X;

			// std::cout << "Plotting map point at " << loc[0] << "," << loc[1] << "," << loc[2] << std::endl;
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
		command += "],c='" + mapColor + "')";

		PyRun_SimpleString(command.c_str());
		// std::cout << "Finished plotting map" << std::endl;
		return Success;
	}

	bool Plotter::plotPose(Pose3& pose, uint8_t flag)
	{
		Vec3 position = pose.center();
		Mat3 rotation = pose.rotation();
		flag ? plotLocation(position, interLocPoseColor) : plotLocation(position, intraLocPoseColor);
		flag ? plotOrientation(rotation, position, interLocPoseColor) : plotOrientation(rotation, position, intraLocPoseColor);

		return Success;
	}

	bool Plotter::plotPoseandCovariance(Pose3& pose, Cov6& cov)
	{
		Vec3 position = pose.center();
		
		std::vector<double> positionCovariance;
		positionCovariance.push_back(cov.at(0)[21]);
		positionCovariance.push_back(cov.at(0)[28]);
		positionCovariance.push_back(cov.at(0)[35]);

		std::string command;
		command = "rx, ry, rz = " +
			std::to_string(positionCovariance[0]) + "," +
			std::to_string(positionCovariance[1]) + "," +
			std::to_string(positionCovariance[2]);
		PyRun_SimpleString(command.c_str());
		PyRun_SimpleString("u = np.linspace(0, 2 * np.pi, 100)");
		PyRun_SimpleString("v = np.linspace(0, np.pi, 100)");
		std::string xLoc = "x = rx * np.outer(np.cos(u), np.sin(v)) + " + std::to_string(position[0]);
		std::string yLoc = "y = ry * np.outer(np.sin(u), np.sin(v)) + " + std::to_string(position[1]);
		std::string zLoc = "z = rz * np.outer(np.ones_like(u), np.cos(v)) + " + std::to_string(position[2]);

		PyRun_SimpleString(xLoc.c_str());
		PyRun_SimpleString(yLoc.c_str());
		PyRun_SimpleString(zLoc.c_str());
		PyRun_SimpleString("axes.plot_surface(x, y, z,  rstride=4, cstride=4, color=None, alpha=0.2)");

		plotLocation(position, intraLocPoseColor);

		// std::cout << "Plotted pose and covariance" << std::endl;
		return true;
	}

	bool Plotter::plotScene(Scene& scene)
	{
		for (auto pose : scene.poses) {
			Pose3 currentPose = pose.second;

			Mat3 R = currentPose.rotation();
			Vec3 t = currentPose.center();

			plotLocation(t, seedPoseColor);
			std::cout << "Plotting seed pose at " << t[0] << "," << t[1] << "," << t[2] << std::endl;
		}

		plotMap(scene);
		return true;
	}

	void Plotter::drawPlot()
	{
		PyRun_SimpleString("plt.show()");
	}
}
