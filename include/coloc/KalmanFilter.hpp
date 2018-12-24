#pragma once

#include "colocData.hpp"
#include <opencv2/video/tracking.hpp>

namespace coloc
{
	class colocFilter
	{
	public:
		std::vector <cv::KalmanFilter> droneFilters;
		std::vector <cv::Mat> droneMeasurements;

		colocFilter(unsigned int& nDrones)
		{
			for (unsigned int i = 0; i < nDrones; ++i) {
				cv::Mat measurements(nMeasurements, 1, CV_64F); measurements.setTo(cv::Scalar(0));
				cv::KalmanFilter KF;
				initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);
				droneFilters.push_back(KF);
				droneMeasurements.push_back(measurements);
			}
		}

		void fillMeasurements(cv::Mat &measurements, const Vec3& translation_measured, const Mat3& rotation_measured)
		{
			// Convert rotation matrix to euler angles
			cv::Mat measured_eulers(3, 1, CV_64F), rotMatrix;

			cv::eigen2cv(rotation_measured, rotMatrix);
			measured_eulers = coloc::Utils::rot2euler(rotMatrix);

			// Set measurement to predict
			measurements.at<double>(0) = translation_measured[0]; // x
			measurements.at<double>(1) = translation_measured[1]; // y
			measurements.at<double>(2) = translation_measured[2]; // z
			measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
			measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
			measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw

			measurementsAvailable = true;
		}

		void update(int& droneId, Pose3& pose, float& rmse)
		{
			cv::Mat predicted, estimated; 
			predicted = droneFilters[droneId].predict();

			cv::setIdentity(droneFilters[droneId].measurementNoiseCov, cv::Scalar::all(rmse/1000));

			if (measurementsAvailable) {
				estimated = droneFilters[droneId].correct(droneMeasurements[droneId]);
				bool reject = chiSquareGating(droneId, droneMeasurements[droneId], predicted);
			}
			else
				estimated = predicted;			

			Vec3 t;
			t[0] = estimated.at<double>(0);
			t[1] = estimated.at<double>(1);
			t[2] = estimated.at<double>(2);

			cv::Mat eulers_estimated(3, 1, CV_64F);
			eulers_estimated.at<double>(0) = estimated.at<double>(9);
			eulers_estimated.at<double>(1) = estimated.at<double>(10);
			eulers_estimated.at<double>(2) = estimated.at<double>(11);

			cv::Mat Rcv = coloc::Utils::euler2rot(eulers_estimated);

			Mat3 R;
			cv::cv2eigen(Rcv, R);

			pose = Pose3(R, t);
			measurementsAvailable = false;
		}

	private:
		int nStates = 18;            
		int nMeasurements = 6;       
		int nInputs = 0;             
		double dt = 0.125;   
		bool measurementsAvailable;

		void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
		{
			KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
			cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
			cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-3));   // set measurement noise
			cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance

			KF.transitionMatrix.at<double>(0, 3) = dt;
			KF.transitionMatrix.at<double>(1, 4) = dt;
			KF.transitionMatrix.at<double>(2, 5) = dt;
			KF.transitionMatrix.at<double>(3, 6) = dt;
			KF.transitionMatrix.at<double>(4, 7) = dt;
			KF.transitionMatrix.at<double>(5, 8) = dt;
			KF.transitionMatrix.at<double>(0, 6) = 0.5*pow(dt, 2);
			KF.transitionMatrix.at<double>(1, 7) = 0.5*pow(dt, 2);
			KF.transitionMatrix.at<double>(2, 8) = 0.5*pow(dt, 2);

			KF.transitionMatrix.at<double>(9, 12) = dt;
			KF.transitionMatrix.at<double>(10, 13) = dt;
			KF.transitionMatrix.at<double>(11, 14) = dt;
			KF.transitionMatrix.at<double>(12, 15) = dt;
			KF.transitionMatrix.at<double>(13, 16) = dt;
			KF.transitionMatrix.at<double>(14, 17) = dt;
			KF.transitionMatrix.at<double>(9, 15) = 0.5*pow(dt, 2);
			KF.transitionMatrix.at<double>(10, 16) = 0.5*pow(dt, 2);
			KF.transitionMatrix.at<double>(11, 17) = 0.5*pow(dt, 2);

			KF.measurementMatrix.at<double>(0, 0) = 1;  // x
			KF.measurementMatrix.at<double>(1, 1) = 1;  // y
			KF.measurementMatrix.at<double>(2, 2) = 1;  // z
			KF.measurementMatrix.at<double>(3, 9) = 1;  // roll
			KF.measurementMatrix.at<double>(4, 10) = 1; // pitch
			KF.measurementMatrix.at<double>(5, 11) = 1; // yaw
		}

		bool chiSquareGating(int& droneId, cv::Mat& estimated, cv::Mat& predicted)
		{
			cv::Mat innv(6, 1, CV_64FC1);
			cv::Mat mDist(1, 1, CV_64FC1);
			cv::Mat S;

			innv.at<double>(0, 0) = estimated.at<double>(0) - predicted.at<double>(0);
			innv.at<double>(1, 0) = estimated.at<double>(1) - predicted.at<double>(1);
			innv.at<double>(2, 0) = estimated.at<double>(2) - predicted.at<double>(2);
			innv.at<double>(3, 0) = estimated.at<double>(3) - predicted.at<double>(3);
			innv.at<double>(4, 0) = estimated.at<double>(4) - predicted.at<double>(4);
			innv.at<double>(5, 0) = estimated.at<double>(5) - predicted.at<double>(5);

			S = (droneFilters[droneId].measurementMatrix * droneFilters[droneId].errorCovPre * droneFilters[droneId].measurementMatrix.t()) + droneFilters[droneId].measurementNoiseCov;

			mDist = innv.t() * S * innv;

			Eigen::MatrixXd Seig, innveig, estimateeig;

			cv::cv2eigen(S, Seig);
			cv::cv2eigen(innv, innveig);
			cv::cv2eigen(estimated, estimateeig);

			std::cout << mDist.at<double>(0, 0) << std::endl;

			std::ofstream myfile;
			myfile.open("mahalanobis.txt", std::ios_base::app);

			myfile << droneId << "," << mDist.at<double>(0,0) << std::endl;

			myfile.close();

			return true;
		}
	};
}