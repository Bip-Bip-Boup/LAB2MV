#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "ReadWriteFunctions.h"


#define STEREO_PARAMS_FILENAME "./data/stereo_params.xml"
#define COLOR_PARAMS_FILENAME "./data/color_params.xml"
#define FPS 30.0
#define STRUCTURAL_ELEMENTS_SIZE 5
#define CAM_LEFT_INDEX 2
#define CAM_RIGHT_INDEX 0
#define RESOLUTION_MAX 600


int main(int argc, char** argv)
{	
	// initializes main parameters
	std::string sStereoCameraParamFilename = STEREO_PARAMS_FILENAME;
	std::string sColorParamFilename = COLOR_PARAMS_FILENAME;
	float fFPS = FPS;
	int iStructuralElementSize = STRUCTURAL_ELEMENTS_SIZE;
	int iCamLIndex = CAM_LEFT_INDEX;
	int iCamRIndex = CAM_RIGHT_INDEX;
	
	// updates main parameters from arguments
	int opt;
	while ((opt = getopt (argc, argv, ":i:o:f:s:l:r:")) != -1)
	{
		switch (opt)
		{
			case 'i':
				sStereoCameraParamFilename = optarg;
				break;
			case 'o':
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
			case '?':
				if (optopt == 'o' || optopt == 'f' || optopt == 'l' || optopt == 'r' || optopt == 'i' || optopt == 's')
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
				else if (isprint (optopt))
					fprintf (stderr, "Unknown option `-%c'.\n", optopt);
				else
					fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
				return 1;
			default:
				abort ();
		}
	}
	
	// distorted/undistorted image
	bool bIsImageUndistorted = true;
	
	// reads camera intrinsic parameters
	cv::Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR;
	cv::Mat stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y, Q;
	bool isCamParamsSet = readStereoCameraParameters(sStereoCameraParamFilename, cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y, Q);
	
	// checks if the camera parameters were successfully read
	if (!isCamParamsSet)
	{
		std::cout << "[WARNING] Camera intrinsic parameters could not be loaded!" << std::endl;
	}
	
	// creates a camera grabber for each camera left/right
	cv::VideoCapture camL(iCamLIndex, cv::CAP_V4L2), camR(iCamRIndex, cv::CAP_V4L2);
	std::cout << "[INFO] Opening camera videostreams...";
	
	// changes image resolution to maximum i.e. 1920x1080 if possible
	camL.set(cv::CAP_PROP_FRAME_HEIGHT, RESOLUTION_MAX); camL.set(cv::CAP_PROP_FRAME_WIDTH, RESOLUTION_MAX);
	camR.set(cv::CAP_PROP_FRAME_HEIGHT, RESOLUTION_MAX); camR.set(cv::CAP_PROP_FRAME_WIDTH, RESOLUTION_MAX);
	
	// checks if the left camera was successfully opened
	if (!camL.isOpened()) // if not success, exit program
	{
		std::cout << std::endl;
		std::cout << "[ERROR] Could not open the left camera!" << std::endl;
		return 1;
	}

	// checks if the right camera was successfully opened
	if (!camR.isOpened()) // if not success, exit program
	{
		std::cout << std::endl;
		std::cout << "[ERROR] Could not open the right camera!" << std::endl;
		return 2;
	}
	std::cout << "OK!" << std::endl;
	
	// gets image resolution for info
	std::cout << "[INFO] Left camera resolution: " << camL.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camL.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << "[INFO] Right camera resolution: " << camR.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camR.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	
	// inits display
	cv::namedWindow("Control", cv::WINDOW_AUTOSIZE); //create a window called "Control"
	cv::namedWindow("Thresholded frames", cv::WINDOW_NORMAL);
	cv::resizeWindow("Thresholded frames", 1200, 600);
	cv::namedWindow("Original frames", cv::WINDOW_NORMAL);
	cv::resizeWindow("Original frames", 1200, 600);

	// sets min/max value for HSV color representation
	int iLowH = 0; int iHighH = 179;
	int iLowS = 0; int iHighS = 255;
	int iLowV = 0; int iHighV = 255;

	// creates trackbars in "Control" window
	cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cv::createTrackbar("HighH", "Control", &iHighH, 179);

	cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cv::createTrackbar("HighS", "Control", &iHighS, 255);

	cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cv::createTrackbar("HighV", "Control", &iHighV, 255);

	while (true)
	{
		cv::Mat frameL, frameR;
		
		// reads a new frame from left/right cameras
		bool bLCamSuccess = camL.read(frameL);
		bool bRCamSuccess = camR.read(frameR);

		// checks if a new frame was grabbed for both cameras
		if (!bLCamSuccess || !bRCamSuccess) //if not success, break loop
		{
			std::cout << "[WARNING] Could not read a both frames from video streams" << std::endl;
			break;
		}
		
		if (bIsImageUndistorted && isCamParamsSet)
		{
			cv::Mat tempL = frameL.clone();
			cv::undistort(tempL, frameL, cameraMatrixL, distCoeffsL);
			cv::Mat tempR = frameR.clone();
			cv::undistort(tempR, frameR, cameraMatrixR, distCoeffsR);
		}
		
		//Convert the captured frame from BGR to HSV
		cv::Mat frameHSVL, frameHSVR;
		cvtColor(frameL, frameHSVL, cv::COLOR_BGR2HSV); 
		cvtColor(frameR, frameHSVR, cv::COLOR_BGR2HSV); 

		//Threshold the image based on the trackbar values
		cv::Mat frameThresholdedL, frameThresholdedR;
		inRange(frameHSVL, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), frameThresholdedL); 
		inRange(frameHSVR, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), frameThresholdedR); 

		//morphological opening (remove small objects from the foreground)
		cv::erode(frameThresholdedL, frameThresholdedL, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) );
		cv::dilate( frameThresholdedL, frameThresholdedL, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) ); 
		cv::erode(frameThresholdedR, frameThresholdedR, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) );
		cv::dilate( frameThresholdedR, frameThresholdedR, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) ); 

		//morphological closing (fill small holes in the foreground)
		cv::dilate( frameThresholdedL, frameThresholdedL, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) ); 
		cv::erode(frameThresholdedL, frameThresholdedL, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) );
		cv::dilate( frameThresholdedR, frameThresholdedR, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) ); 
		cv::erode(frameThresholdedR, frameThresholdedR, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) );
		
		// displays the thresholded L/R frames
		cv::Mat tempThres;
		cv::hconcat(frameThresholdedL, frameThresholdedR, tempThres);
		cv::imshow("Thresholded frames", tempThres); //show the thresholded image
		
		// displays the original L/R frames
		cv::Mat temp;
		cv::hconcat(frameL, frameR, temp);
		cv::imshow("Original frames", temp); 

		// waits for awhile depending on the FPS value
		// checks if ESC was pressed to exit
		char key = (char)cv::waitKey(1000.0/fFPS);
		
		//wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		if (key == 27)
		{
			std::cout << "[INFO] esc key is pressed by user -> Shutting down!" << std::endl;
			break; 
		}
		if (key == 'u')
		{
			bIsImageUndistorted = !bIsImageUndistorted;
			std::cout << "[INFO] Image undistorted: " <<  bIsImageUndistorted<< std::endl;
		}
		if (key == 's')
		{
			std::cout << "[INFO] Color parameters saved to file... ";
			bool isColorParamsSaved = writeColorParameters(sColorParamFilename, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
			// checks if the color parameters were successfully read
			if (!isColorParamsSaved)
			{
				std::cout << std::endl;
				std::cout << "\t[ERROR] Color parameters could not be saved!" << std::endl;
				return 4;
			}
			std::cout << "OK!" << std::endl;
		}
		
	}
	
	// releases video streams
	camL.release();
	camR.release();
	
	// destroys all windows
	cv::destroyAllWindows();

	return 0;
}
