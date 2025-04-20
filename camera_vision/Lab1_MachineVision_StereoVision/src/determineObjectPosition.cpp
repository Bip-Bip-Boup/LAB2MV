#include <iostream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/calib3d.hpp>

#include "ReadWriteFunctions.h"
#include "ImageProcessing.h"

#define STEREO_PARAMS_FILENAME "./data/stereo_params.xml"
#define COLOR_PARAMS_FILENAME "./data/color_params.xml"
#define DISPARITY_PARAMS_FILENAME "./data/disparity_params.xml"
#define FPS 30.0
#define STRUCTURAL_ELEMENTS_SIZE 5
#define CAM_LEFT_INDEX 2
#define CAM_RIGHT_INDEX 0
#define AREA_THRESOLD 1000
#define RESOLUTION_MAX 2000
#define DISPARITY_WIN_SIZE 720



int main( int argc, char** argv )
{
	// initializes main parameters
	std::string sStereoCameraParamFilename = STEREO_PARAMS_FILENAME;
	std::string sDisparityParamFilename = DISPARITY_PARAMS_FILENAME;
	std::string sColorParamFilename = COLOR_PARAMS_FILENAME;
	float fFPS = FPS;
	int iStructuralElementSize = STRUCTURAL_ELEMENTS_SIZE;
	int iCamLIndex = CAM_LEFT_INDEX;
	int iCamRIndex = CAM_RIGHT_INDEX;
	int iAreaThresold = AREA_THRESOLD;
	 
	// updates main parameters from arguments
	int opt;
	while ((opt = getopt (argc, argv, ":i:d:c:f:s:l:r:a:")) != -1)
	{
		switch (opt)
		{
			case 'i':
				sStereoCameraParamFilename = optarg;
				break;
			case 'd':
				sDisparityParamFilename = optarg;
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
				return 1;
			default:
				abort ();
		}
	}
	
	// creates an object of StereoBM algorithm
	cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
	
	// inits values for StereoSGBM parameters
	int numDisparities = 64;  
	int blockSize = 21;
	int preFilterType = 1;
	int preFilterSize = 7;
	int preFilterCap = 31;
	int minDisparity = 0;
	int textureThreshold = 10; 
	int uniquenessRatio = 10;
	int speckleRange = 8;
	int speckleWindowSize = 20;
	int disp12MaxDiff = 1;
	
	// reads values from file if it exists
	bool isDisparityParamsSet = readDisparityParameters(sDisparityParamFilename, numDisparities, blockSize , preFilterType, preFilterSize, preFilterCap, minDisparity , textureThreshold, uniquenessRatio, speckleRange, speckleWindowSize, disp12MaxDiff);
	
	// checks if the disparity parameters were successfully read
	if (!isDisparityParamsSet)
	{
		std::cout << "[ERROR] Disparity parameters could not be loaded!" << std::endl;
		return -1;
	}
	
	// fills the SGBM algo with params
	stereo->setNumDisparities(numDisparities);
	stereo->setBlockSize(blockSize);
	stereo->setPreFilterType(preFilterType);
	stereo->setPreFilterSize(preFilterSize);
	stereo->setPreFilterCap(preFilterCap);
	stereo->setTextureThreshold(textureThreshold);
	stereo->setUniquenessRatio(uniquenessRatio);
	stereo->setSpeckleRange(speckleRange);
	stereo->setSpeckleWindowSize(speckleWindowSize);
	stereo->setDisp12MaxDiff(disp12MaxDiff);
	stereo->setMinDisparity(minDisparity);
	
	// reads color parameters from the file storage
	int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
	bool isColorParamsSet = readColorParameters(sColorParamFilename, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
	
	// checks if the color parameters were successfully read
	if (!isColorParamsSet)
	{
		std::cout << "[ERROR] Color parameters could not be loaded!" << std::endl;
		return 1;
	}
	
	// distorted/undistorted image
	bool bIsImageUndistorted = true;
	
	// reads camera intrinsic parameters
	cv::Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR;
	cv::Mat stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y, Q;
	bool isCamParamsSet = readStereoCameraParameters(sStereoCameraParamFilename, cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y, Q);
	
	// checks if the stereo camera parameters were successfully read
	if (!isCamParamsSet)
	{
		std::cout << "[ERROR] Stereo camera parameters could not be loaded!" << std::endl;
		return 2;
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
		return 3;
	}

	// checks if the right camera was successfully opened
	if (!camR.isOpened()) // if not success, exit program
	{
		std::cout << std::endl;
		std::cout << "[ERROR] Could not open the right camera!" << std::endl;
		return 4;
	}
	std::cout << "OK!" << std::endl;
	
	// gets image resolution for info
	std::cout << "[INFO] Left camera resolution: " << camL.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camL.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << "[INFO] Right camera resolution: " << camR.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camR.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

	// inits previous x,y location of the ball
	int iLastXR = -1; int iLastYR = -1;
	int iLastXL = -1; int iLastYL = -1;
	
	// captures a temporary image from the camera
	cv::Mat imgTmp;
	camL.read(imgTmp); 

	// creates a black image with the size as the camera output
	cv::Mat imgLinesL = cv::Mat::zeros( imgTmp.size(), CV_8UC3 );
	cv::Mat imgLinesR = cv::Mat::zeros( imgTmp.size(), CV_8UC3 );
	
	// inits display
	cv::namedWindow("Original", cv::WINDOW_NORMAL);
	cv::resizeWindow("Original", 1200, 600);
	cv::namedWindow("Thresholded", cv::WINDOW_NORMAL);
	cv::resizeWindow("Thresholded", 1200, 600);
	cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
	cv::resizeWindow("Disparity", 1000, 1000);

	// main loop launched every FPS
	while (true)
	{
		cv::Mat frameL, frameR;
		cv::Mat frameThresholdedL, frameThresholdedR;
		cv::Mat frameGrayL, frameGrayR;
		cv::Mat frameNiceL, frameNiceR;
		cv::Mat disp, disparity;
		
		// reads a new frame from left/right cameras
		bool bLCamSuccess = camL.read(frameL);
		bool bRCamSuccess = camR.read(frameR);

		// checks if a new frame was grabbed for both cameras
		if (!bLCamSuccess || !bRCamSuccess) //if not success, break loop
		{
			std::cout << "[WARNING] Could not read a both frames from video streams" << std::endl;
			break;
		}
		
		// finds the object position in pixels
		findObject2DPosition(frameL, frameThresholdedL, bIsImageUndistorted, isCamParamsSet, cameraMatrixL, distCoeffsL, iLowH, iLowS, iLowV, iHighH, iHighS, iHighV, iStructuralElementSize, iAreaThresold, iLastXL, iLastYL, imgLinesL);
		findObject2DPosition(frameR, frameThresholdedR, bIsImageUndistorted, isCamParamsSet, cameraMatrixR, distCoeffsR, iLowH, iLowS, iLowV, iHighH, iHighS, iHighV, iStructuralElementSize, iAreaThresold, iLastXR, iLastYR, imgLinesR);
		
		// draws markers
		cv::drawMarker(frameThresholdedL, cv::Point(iLastXL,iLastYL), cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 10, 1);  
		cv::drawMarker(frameThresholdedR, cv::Point(iLastXR,iLastYR), cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 10, 1);  
		
		// converts frames to grayscale
		cv::cvtColor(frameL, frameGrayL, cv::COLOR_BGR2GRAY);
		cv::cvtColor(frameR, frameGrayR, cv::COLOR_BGR2GRAY);

		// applies stereo image rectification on the both images
		cv::remap(frameGrayL, frameNiceL, stereoMapL_x, stereoMapL_y, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
		cv::remap(frameGrayR, frameNiceR, stereoMapR_x, stereoMapR_y, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

		// resizes left and right rectified images to be squared (works better with SGBM algo
		cv::resize(frameNiceL, frameNiceL,cv::Size(DISPARITY_WIN_SIZE, DISPARITY_WIN_SIZE));
		cv::resize(frameNiceR, frameNiceR,cv::Size(DISPARITY_WIN_SIZE, DISPARITY_WIN_SIZE));
		
		// calculates disparity using the StereoBM algorithm
		if (numDisparities!= 0)
			stereo->compute(frameNiceL,frameNiceR,disp);
		
		// converts disparity values to CV_32F from CV_16S
		// NOTE: compute returns a 16bit signed single channel image,
		// CV_16S containing a disparity map scaled by 16. Hence it 
		// is essential to convert it to CV_32F and scale it down 16 times.
		// converts disparity values to CV_32F from CV_16S
		disp.convertTo(disparity, CV_32F, 1.0/16);
		
		// normalizes disparity map for display purposes
		cv::normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);  
		
		// displays disparity map with a cross at the object position
		float v=((float)iLastYR + (float)iLastYL) / 2 ;
		float u=((float)iLastXR + (float)iLastXL + numDisparities) / 2 ;
	
		v /= frameGrayL.rows;
		v *= DISPARITY_WIN_SIZE;
		u /= frameGrayL.cols;
		u *= DISPARITY_WIN_SIZE;
			
		cv::drawMarker(disp, cv::Point(u,v), cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 10, 1);  
		cv::imshow("Disparity",disp);
		
		// finds the object 3D position
		double x, y, z;
		findObject3DPosition( iLastXR, iLastYR, iLastXL, iLastYL, frameGrayL, disparity, Q, x, y, z, numDisparities);
		
		std::cout << "XYZ_cm= (" << x/10 << ", " << y/10 << ", " << z/10 <<")"<< std::endl;
		
		// displays the thresholded L/R frames
		cv::Mat tempThres;
		cv::hconcat(frameThresholdedL, frameThresholdedR, tempThres);
		cv::imshow("Thresholded", tempThres); //show the thresholded image
		
		// shows the original image with the tracking (red) lines
		frameL = frameL + imgLinesL;
		frameR = frameR + imgLinesR;
		cv::Mat temp;
		cv::hconcat(frameL, frameR, temp);
		imshow("Original", temp); 

		// waits for awhile depending on the FPS value
		char key = (char)cv::waitKey(1000.0/fFPS);
		// checks if ESC was pressed to exit
		if (key == 27) // if 'esc' key is pressed, break loop
		{
			std::cout << "[INFO] Esc key is pressed by user -> Shutting down!" << std::endl;
			break; 
		}
		if (key == 'u')
		{
			bIsImageUndistorted = !bIsImageUndistorted;
			std::cout << "[INFO] Image undistorted: " <<  bIsImageUndistorted<< std::endl;
		}
	}

	return 0;
}