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

// global vars
std::string sStereoCameraParamFilename = STEREO_PARAMS_FILENAME;
std::string sColorParamFilename = COLOR_PARAMS_FILENAME;
int iStructuralElementSize = STRUCTURAL_ELEMENTS_SIZE;
int iAreaThresold = AREA_THRESOLD;
int iCamLIndex = CAM_LEFT_INDEX;
int iCamRIndex = CAM_RIGHT_INDEX;
float fFPS = FPS;


bool processArgs(int argc, char** argv)
{
	int opt;
	while ((opt = getopt (argc, argv, ":i:c:f:s:l:r:a:")) != -1)
	{
		switch (opt)
		{
			case 'i':
				sStereoCameraParamFilename = optarg;
				break;
			case 'c':
				sColorParamFilename = optarg;
				break;
			case 'f':
				fFPS = atof(optarg); 
				break;
			case 's':
				iStructuralElementSize = atoi(optarg);
				break;
			case 'l':
				iCamLIndex = atoi(optarg);
				break;
			case 'r':
				iCamRIndex = atoi(optarg);
				break;
			case 'a':
				iAreaThresold = atoi(optarg);
				break;
			case '?':
				if (optopt == 'i' || optopt == 'd' || optopt == 'c' || optopt == 'f' || optopt == 's' || optopt == 'l' || optopt == 'r' || optopt == 'a')
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
				else if (isprint (optopt))
					fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				else
					fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
				return false;
			default:
				abort ();
		}
	}
	return true;
}


int main(int argc, char** argv)
{
	// random values
	srand(time(NULL));
	
	// robot vars
	DynamixelHandler _oDxlHandler;
	std::string _robotDxlPortName = ROBOT_PORT_NAME;
	float _robotDxlProtocol = ROBOT_PROTOCOL;
	int _robotDxlBaudRate = ROBOT_BAUDRATE;
	float L1 = ROBOT_L1;
	float L2 = ROBOT_L2;
	float L3 = ROBOT_L3;
	float q1 = ROBOT_Q1;
	float q2 = deg2rad(ROBOT_Q2);
	float q3 = ROBOT_Q3;
	float qgripper = ROBOT_GRIPPER;
	
	// color cars
	int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
	
	// stereocam params
	cv::Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR;
	cv::Mat stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y, Q;
	
	// robot/cam params
	cv::Mat robotCamTransform;
	
	// output file
	std::string sCamRobotCalibrationFilename = CAM_ROBOT_CALIB_FILENAME;
	
	std::cout << "========INITIALISATION PHASE========" <<std::endl;
	
	// updates main parameters from arguments
	std::cout << "[INFO] Process args... ";
	bool isArgSet = processArgs(argc, argv);
	if (!isArgSet)
	{
		std::cout << std::endl;
		std::cout << "\t[ERROR] Args could not be set!" << std::endl;
		return 1;
	}
	std::cout << "OK!" << std::endl;
	
	// initializes the robot
	std::cout << "[INFO] Initialize the robot" << std::endl;
	initRobot(_oDxlHandler, _robotDxlPortName, _robotDxlProtocol, _robotDxlBaudRate);
	
	// reads color parameters from the file storage
	std::cout << "[INFO] Read color parameters... ";
	bool _isColorParamsSet = readColorParameters(sColorParamFilename, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
	if (!_isColorParamsSet)
	{
		std::cout << std::endl;
		std::cout << "\t[ERROR] Color parameters could not be loaded!" << std::endl;
		return 2;
	}
	std::cout << "OK!" << std::endl;
	
	// reads camera intrinsic & extrinsic parameters
	std::cout << "[INFO] Read Stereo parameters... ";
	bool _isCamParamsSet = readStereoCameraParameters(sStereoCameraParamFilename, cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y, Q);
	if (!_isCamParamsSet)
	{
		std::cout << std::endl;
		std::cout << "\t[ERROR] Stereo camera parameters could not be loaded!" << std::endl;
		return 3;
	}
	std::cout << "OK!" << std::endl;
	
	// reads camera/robot calibration parameters
	bool _isRobotCamParamsSet = readCamRobotCalibrationParameters(sCamRobotCalibrationFilename, robotCamTransform);
	if (!_isRobotCamParamsSet)
	{
		std::cout << std::endl;
		std::cout << "\t[ERROR] Robot/Camera calibration parameters could not be loaded!" << std::endl;
		return 3;
	}
	std::cout << "OK!" << std::endl;
	
	// creates a camera grabber for each camera left/right
	std::cout << "[INFO] Opening camera videostreams...";
	
	cv::VideoCapture camL(iCamLIndex, cv::CAP_V4L2), camR(iCamRIndex, cv::CAP_V4L2);
	
	// changes image resolution to maximum i.e. 1920x1080 if possible
	camL.set(cv::CAP_PROP_FRAME_HEIGHT, RESOLUTION_MAX); camL.set(cv::CAP_PROP_FRAME_WIDTH, RESOLUTION_MAX);
	camR.set(cv::CAP_PROP_FRAME_HEIGHT, RESOLUTION_MAX); camR.set(cv::CAP_PROP_FRAME_WIDTH, RESOLUTION_MAX);
	
	// checks if the left camera was successfully opened
	if (!camL.isOpened()) // if not success, exit program
	{
		std::cout << std::endl;
		std::cout << "\t[ERROR] Could not open the left camera!" << std::endl;
		return 4;
	}

	// checks if the right camera was successfully opened
	if (!camR.isOpened()) // if not success, exit program
	{
		std::cout << std::endl;
		std::cout << "\t[ERROR] Could not open the right camera!" << std::endl;
		return 5;
	}
	std::cout << "OK!" << std::endl;
	
	// gets image resolution for info
	std::cout << "\t-->Left camera resolution: " << camL.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camL.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << "\t-->Right camera resolution: " << camR.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camR.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	
	// inits display
	cv::namedWindow("Original", cv::WINDOW_NORMAL);
	cv::resizeWindow("Original", 1280, 480);
	cv::namedWindow("Thresholded", cv::WINDOW_NORMAL);
	cv::resizeWindow("Thresholded", 1280, 480);
	
	// inits previous x,y location of the ball
	int iLastXR = -1; int iLastYR = -1;
	int iLastXL = -1; int iLastYL = -1;
	
	// captures a temporary image from the camera
	cv::Mat imgTmp;
	camL.read(imgTmp); 

	// creates a black image with the size as the camera output
	cv::Mat imgLinesL = cv::Mat::zeros( imgTmp.size(), CV_8UC3 );
	cv::Mat imgLinesR = cv::Mat::zeros( imgTmp.size(), CV_8UC3 );
	
	std::cout << "========PLACE THE CUBE IN THE ROBOT GRIPPER========" <<std::endl;
	
	// sends the target joint values received as args
	std::cout << "[INFO] Move to the initial position"<< std::endl;
	moveJoint(_oDxlHandler, q1, q2, q3, qgripper);
	std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // waits 2s
	
	std::cout << "========TRACKING PHASE========" <<std::endl;
	// main loop launched every FPS
	while (true)
	{
		cv::Mat frameL, frameR;
		cv::Mat frameThresholdedL, frameThresholdedR;
		cv::Mat frameNiceL, frameNiceR;
		
		// reads a new frame from left/right cameras
		bool bLCamSuccess = camL.read(frameL);
		bool bRCamSuccess = camR.read(frameR);

		// checks if a new frame was grabbed for both cameras
		if (!bLCamSuccess || !bRCamSuccess) //if not success, break loop
		{
			std::cout << "[WARNING] Could not read both frames from video streams" << std::endl;
			break;
		}
		
		// applies stereo image rectification on both images
		cv::remap(frameL, frameNiceL, stereoMapL_x, stereoMapL_y, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
		cv::remap(frameR, frameNiceR, stereoMapR_x, stereoMapR_y, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

		// finds the object position in pixels
		findObject2DPosition(frameNiceL, frameThresholdedL, iLowH, iLowS, iLowV, iHighH, iHighS, iHighV, iStructuralElementSize, iAreaThresold, iLastXL, iLastYL, imgLinesL);
		findObject2DPosition(frameNiceR, frameThresholdedR, iLowH, iLowS, iLowV, iHighH, iHighS, iHighV, iStructuralElementSize, iAreaThresold, iLastXR, iLastYR, imgLinesR);
	
		// draws markers
		cv::drawMarker(frameThresholdedL, cv::Point(iLastXL,iLastYL), cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 10, 1);  
		cv::drawMarker(frameThresholdedR, cv::Point(iLastXR,iLastYR), cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 10, 1);  
			
		// finds the object 3D position in the {camera} frame
		double x, y, z;
		findObject3DPosition( iLastXR, iLastYR, iLastXL, iLastYL, frameThresholdedL, Q, x, y, z);
			
		// determines the object 3D position in the {robot} frame
		cv::Mat_<double> ptsCameraFrame(4,1);
		ptsCameraFrame(0) = x/10;
		ptsCameraFrame(1) = y/10;
		ptsCameraFrame(2) = z/10;
		ptsCameraFrame(3) = 1;
		std::cout << "XYZ_{camera}= " << ptsCameraFrame<< std::endl;
		cv::Mat_<double> ptsRobotFrame(4,1);
		ptsRobotFrame = robotCamTransform*ptsCameraFrame;
		std::cout << "XYZ_{robot}= " << ptsRobotFrame << std::endl;
		
		// applies IK
		std::vector<float> qi = computeInverseKinematics(ptsRobotFrame(0), ptsRobotFrame(1), ptsRobotFrame(2), L1, L2, L3);
		
		if (qi[0] == 2)
		{
			std::cout << "qi2= (" << qi[4] << ", " << qi[5] << ", " << qi[6] << ")" << std::endl;
			moveJoint(_oDxlHandler, qi[4] , qi[5], qi[6], 0);
		}
		if (qi[0] == 1)
		{
			std::cout << "qi1= (" << qi[1] << ", " << qi[2] << ", " << qi[3] << ")" << std::endl;
			moveJoint(_oDxlHandler, qi[1] , qi[2], qi[3], 0);
		}
		

		// displays the thresholded L/R frames
		cv::Mat tempThres;
		cv::hconcat(frameThresholdedL, frameThresholdedR, tempThres);
		cv::imshow("Thresholded", tempThres); //show the thresholded image
		
		// shows the original image with the tracking (red) lines
		frameNiceL = frameNiceL + imgLinesL;
		frameNiceR = frameNiceR + imgLinesR;
		cv::Mat temp;
		cv::hconcat(frameNiceL, frameNiceR, temp);
		imshow("Original", temp); 
				
		// waits for awhile depending on the FPS value
		char key = (char)cv::waitKey(1000.0/fFPS);
		
		// checks if ESC was pressed to exit
		if (key == 27) // if 'esc' key is pressed, break loop
		{
			std::cout << "[INFO] Esc key is pressed by user -> Shutting down without calibrating!" << std::endl;
			break; 
		}
		
	}
	
	std::cout << "========CLOSING PHASE========" <<std::endl;
	
	// Closes robot connection
	std::cout << "[INFO] Closing the robot connection"<< std::endl;
	closeRobot(_oDxlHandler);
	
	
	// releases video streams
	camL.release();
	camR.release();
	
	// destroys all windows
	cv::destroyAllWindows();
	
	return 0;
}
