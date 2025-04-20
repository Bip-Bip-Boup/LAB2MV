#include <chrono>
#include <thread>

#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d.hpp>

#include "ReadWriteFunctions.h"
#include "ImageProcessing.h"
#include "Kinematics.h"
#include "DynamixelHandler.h"
#include "RobotControl.h"
#include "Utils.h"

#define STEREO_PARAMS_FILENAME "./data/stereo_params.xml"
#define COLOR_PARAMS_FILENAME "./data/color_params.xml"
#define CAM_ROBOT_CALIB_FILENAME "./data/camrobot_params.xml"

#define FPS 30.0
#define STRUCTURAL_ELEMENTS_SIZE 5
#define CAM_LEFT_INDEX 2
#define CAM_RIGHT_INDEX 0
#define AREA_THRESOLD 1000
#define RESOLUTION_MAX 800

#define ROBOT_PORT_NAME "/dev/ttyUSB0"
#define ROBOT_PROTOCOL 2.0
#define ROBOT_BAUDRATE 1000000
#define ROBOT_L1 3.0
#define ROBOT_L2 7.0
#define ROBOT_L3 11.0
#define ROBOT_Q1 0.0
#define ROBOT_Q2 -60.0
#define ROBOT_Q3 0.0
#define ROBOT_GRIPPER 0.0

std::vector<cv::Point> findYellowBlobs(cv::Mat& frameThresholded, int areaThreshold)
{
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> centers;
	cv::findContours(frameThresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	for (const auto& contour : contours)
	{
		if (cv::contourArea(contour) > areaThreshold)
		{
			cv::Moments M = cv::moments(contour);
			if (M.m00 != 0)
			{
				int cx = static_cast<int>(M.m10 / M.m00);
				int cy = static_cast<int>(M.m01 / M.m00);
				centers.push_back(cv::Point(cx, cy));
			}
		}
	}
	return centers;
}

int main(int argc, char** argv)
{
	srand(time(NULL));
	DynamixelHandler _oDxlHandler;
	initRobot(_oDxlHandler, ROBOT_PORT_NAME, ROBOT_PROTOCOL, ROBOT_BAUDRATE);

	float L1 = ROBOT_L1, L2 = ROBOT_L2, L3 = ROBOT_L3;
	float q1 = ROBOT_Q1, q2 = deg2rad(ROBOT_Q2), q3 = ROBOT_Q3, qgripper = ROBOT_GRIPPER;

	int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
	cv::Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR;
	cv::Mat stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y, Q;
	cv::Mat robotCamTransform;

	readColorParameters(COLOR_PARAMS_FILENAME, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
	readStereoCameraParameters(STEREO_PARAMS_FILENAME, cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y, Q);
	readCamRobotCalibrationParameters(CAM_ROBOT_CALIB_FILENAME, robotCamTransform);

	cv::VideoCapture camL(CAM_LEFT_INDEX), camR(CAM_RIGHT_INDEX);
	camL.set(cv::CAP_PROP_FRAME_HEIGHT, RESOLUTION_MAX);
	camL.set(cv::CAP_PROP_FRAME_WIDTH, RESOLUTION_MAX);
	camR.set(cv::CAP_PROP_FRAME_HEIGHT, RESOLUTION_MAX);
	camR.set(cv::CAP_PROP_FRAME_WIDTH, RESOLUTION_MAX);

	cv::Mat imgLinesL, imgLinesR;
	moveJoint(_oDxlHandler, q1, q2, q3, qgripper);
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	int towerIndex = 0;
	while (true)
	{
		cv::Mat frameL, frameR, frameNiceL, frameNiceR, threshL, threshR;
		if (!camL.read(frameL) || !camR.read(frameR)) break;
		cv::remap(frameL, frameNiceL, stereoMapL_x, stereoMapL_y, cv::INTER_LANCZOS4);
		cv::remap(frameR, frameNiceR, stereoMapR_x, stereoMapR_y, cv::INTER_LANCZOS4);

		cv::inRange(frameNiceL, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), threshL);
		cv::inRange(frameNiceR, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), threshR);

		std::vector<cv::Point> centersL = findYellowBlobs(threshL, AREA_THRESOLD);
		std::vector<cv::Point> centersR = findYellowBlobs(threshR, AREA_THRESOLD);
		int numObjects = std::min(centersL.size(), centersR.size());

		for (int i = 0; i < numObjects; ++i)
		{
			int xL = centersL[i].x, yL = centersL[i].y;
			int xR = centersR[i].x, yR = centersR[i].y;

			double x, y, z;
			findObject3DPosition(xR, yR, xL, yL, threshL, Q, x, y, z);

			cv::Mat_<double> camPt(4,1); camPt << x/10, y/10, z/10, 1;
			cv::Mat_<double> robPt = robotCamTransform * camPt;
			std::vector<float> qi = computeInverseKinematics(robPt(0), robPt(1), robPt(2), L1, L2, L3);

			if (qi[0] == 1) {
				moveJoint(_oDxlHandler, qi[1], qi[2], qi[3], 0); // pick
				moveJoint(_oDxlHandler, qi[1], qi[2], qi[3], 1); // close gripper
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			float zStack = 2.0 + towerIndex * 3.0;
			std::vector<float> qStack = computeInverseKinematics(15.0, 0.0, zStack, L1, L2, L3);
			if (qStack[0] == 1) {
				moveJoint(_oDxlHandler, qStack[1], qStack[2], qStack[3], 1); // place
				moveJoint(_oDxlHandler, qStack[1], qStack[2], qStack[3], 0); // release
			}
			moveJoint(_oDxlHandler, q1, q2, q3, qgripper); // home
			towerIndex++;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		cv::imshow("Thresholded", threshL);
		if ((char)cv::waitKey(1000.0/FPS) == 27) break;
	}

	closeRobot(_oDxlHandler);
	camL.release(); camR.release();
	cv::destroyAllWindows();
	return 0;
}
