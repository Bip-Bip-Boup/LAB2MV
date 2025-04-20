#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <stdio.h>
#include <iostream>
#include <unistd.h>

#include "ReadWriteFunctions.h"

#define STEREO_PARAMS_FILENAME "./data/stereo_params.xml"
#define MONO_LEFT_CALIB_PARAMS_FILENAME "./data/leftCamera_calibration_params.xml"
#define MONO_RIGHT_CALIB_PARAMS_FILENAME "./data/rightCamera_calibration_params.xml"
#define FPS 20
#define CAM_LEFT_INDEX 2
#define CAM_RIGHT_INDEX 0
#define NB_FRAMES 25
#define SQUARE_SIZE 23.4
#define CHECKERBOARD_WIDTH 6
#define CHECKERBOARD_HEIGHT 9
#define RESOLUTION_MAX 800
 

int main( int argc, char** argv )
{
	// initializes main parameters 
	std::string sStereoParamFilename = STEREO_PARAMS_FILENAME;
	float fFPS = FPS;
	int iCamLIndex = CAM_LEFT_INDEX;
	int iCamRIndex = CAM_RIGHT_INDEX;
	int iNbFrames = NB_FRAMES;
	float fSquareSize = SQUARE_SIZE; // in mm
	int iCheckerBoardWidth = CHECKERBOARD_WIDTH;
	int iCheckerBoardHeight = CHECKERBOARD_HEIGHT;
	std::string sMonoLeftCalibParamFilename = MONO_LEFT_CALIB_PARAMS_FILENAME;
	std::string sMonoRightCalibParamFilename = MONO_RIGHT_CALIB_PARAMS_FILENAME;
	int iMaxVideoResolution = RESOLUTION_MAX;
	
	// updates main parameters from arguments
	int opt;
	while ((opt = getopt (argc, argv, ":o:f:l:r:n:s:w:h:i:j:k:")) != -1)
	{
		switch (opt)
		{
			case 'o':
				sStereoParamFilename = optarg;
				break;
			case 'f':
				fFPS = atof(optarg); 
				break;
			case 'l':
				iCamLIndex = atoi(optarg);
				break;
			case 'r':
				iCamRIndex = atoi(optarg);
				break;
			case 'n':
				iNbFrames = atoi(optarg);
				break;
			case 's':
				fSquareSize = atof(optarg);
				break;
			case 'w':
				iCheckerBoardWidth = atoi(optarg);
				break;
			case 'h':
				iCheckerBoardHeight = atoi(optarg);
				break;
			case 'i':
				sMonoLeftCalibParamFilename = optarg;
				break;
			case 'j':
				sMonoRightCalibParamFilename = optarg;
				break;
			case 'k':
				iMaxVideoResolution = atoi(optarg);
				break;
			case '?':
				if (optopt == 'o' || optopt == 'f' || optopt == 'l' || optopt == 'r' || optopt == 'n' || optopt == 's' || optopt == 'w' || optopt == 'h'|| optopt == 'i'|| optopt == 'j'|| optopt == 'k')
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

	// creates camera grabbers for the left and right cameras
	std::cout << "[INFO] Opening camera videostreams...";
	cv::VideoCapture camL(iCamLIndex, cv::CAP_V4L2), camR(iCamRIndex, cv::CAP_V4L2);
	
	// changes image resolution to maximum (e.g. 1920x1080 if possible)
	camL.set(cv::CAP_PROP_FRAME_HEIGHT, iMaxVideoResolution); camL.set(cv::CAP_PROP_FRAME_WIDTH, iMaxVideoResolution);
	camR.set(cv::CAP_PROP_FRAME_HEIGHT, iMaxVideoResolution); camR.set(cv::CAP_PROP_FRAME_WIDTH, iMaxVideoResolution);
	
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
	cv::namedWindow("Frames", cv::WINDOW_NORMAL);
	cv::resizeWindow("Frames", 1280, 480);
	
	// defines the world coordinates for 3D points (in mm)
	std::vector<cv::Point3f> objp;
	for(int i{0}; i<iCheckerBoardHeight; i++)
	{
		for(int j{0}; j<iCheckerBoardWidth; j++)
			objp.push_back(cv::Point3f(j*fSquareSize, i*fSquareSize, 0));
	}
	
	// loads each camera calibration parameters
	cv::Mat camLMatrix, distLCoeffs, camRMatrix, distRCoeffs;
	bool isLeftCamCalibParamsLoaded = readMonoCameraCalibParameters(sMonoLeftCalibParamFilename, camLMatrix, distLCoeffs);
	
	// checks if the file storage is opened before continuing
	if (!isLeftCamCalibParamsLoaded)
	{
		std::cout << "[ERROR] Could not open the mono calibration params file: " <<  sMonoLeftCalibParamFilename << " !"<< std::endl;
		return 3;
	}
	std::cout << "OK!" << std::endl;
	
	bool isRightCamCalibParamsLoaded = readMonoCameraCalibParameters(sMonoRightCalibParamFilename, camRMatrix, distRCoeffs);
	
	// checks if the file storage is opened before continuing
	if (!isRightCamCalibParamsLoaded)
	{
		std::cout << "[ERROR] Could not open the mono calibration params file: " <<  isRightCamCalibParamsLoaded << " !"<< std::endl;
		return 3;
	}
	std::cout << "OK!" << std::endl;
	
	// main loop launched every FPS
	int iCount = 0;
	bool bIsCalibrated = false;
	bool bIsUndistort = false;
	cv::Mat frameL, frameR;
	cv::Mat grayL, grayR;
	cv::Mat viewL, viewR;
	std::vector<std::vector<cv::Point3f> > objpoints; // vector to store vectors of 3D points for each checkerboard image
	std::vector<std::vector<cv::Point2f> > imgpointsL, imgpointsR; // vector to store vectors of 2D points for each checkerboard image
	cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
	cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;
	cv::Mat Q;
	
	while (true)
	{
		// reads a new frame from left & right cameras
		bool bLCamSuccess = camL.read(frameL);
		bool bRCamSuccess = camR.read(frameR);

		// checks if a new frame was grabbed for both cameras
		if (!bLCamSuccess || !bRCamSuccess) //if not success, break loop
		{
			std::cout << "[WARNING] Could not read a both frames from video streams" << std::endl;
			break;
		}
		
		// gets a copy for display
		viewL = frameL.clone();
		viewR = frameR.clone();
		
		// converts frames to grayscale
		cv::cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);
		cv::cvtColor(frameR, grayR, cv::COLOR_BGR2GRAY);
		
		// looks for chessboard in the frames
		std::vector<cv::Point2f> cornersL, cornersR;
		cv::Size board = cv::Size(iCheckerBoardWidth, iCheckerBoardHeight);
		bool foundLR = false;
		bool foundL = cv::findChessboardCorners(grayL, board, cornersL, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		bool foundR = cv::findChessboardCorners(grayR, board, cornersR, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
				
		// checks if chessboards were found in both frames
		if ((foundL==true) && (foundR==true) && !bIsCalibrated)
		{
			foundLR = true;
			cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

			// refines pixel coordinates for given 2d points
			cv::cornerSubPix(grayL, cornersL, cv::Size(11,11), cv::Size(-1,-1), criteria);
			cv::cornerSubPix(grayR, cornersR, cv::Size(11,11), cv::Size(-1,-1), criteria);

			// displays the detected corner points on the checker board
			cv::drawChessboardCorners(viewL, cv::Size(iCheckerBoardWidth, iCheckerBoardHeight), cornersL, foundL);
			cv::drawChessboardCorners(viewR, cv::Size(iCheckerBoardWidth, iCheckerBoardHeight), cornersR, foundR);			
		
			cv::putText(viewL, "Press g to grab the frame", cv::Point(50, 50), 2, 1, cv::Scalar(0, 200, 0), 2);
		}
		
		
		cv::Mat temp;
		if (!bIsCalibrated)
		{
			// concats left & right frames and displays it
			cv::hconcat(viewR, viewL, temp);
			
			// adds frame number and displays it
			std::string sCounter = std::to_string(iCount) + "/" + std::to_string(iNbFrames);
			cv::putText(temp, sCounter, cv::Point(50, 100), 2, 1, cv::Scalar(0, 200, 200), 2);
			
			
		}
		else
		{
			cv::Mat niceL, niceR;
			
			if (bIsUndistort)
			{
				cv::remap(frameL, niceL, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
				cv::remap(frameR, niceR, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
			}
			else
			{
				niceL = frameL.clone();
				niceR = frameR.clone();
			}
	
			cv::hconcat(niceR, niceL, temp);
			
			for (int l=0; l< temp.rows; l+=100) 
				cv::line(temp, cv::Point(0,l), cv::Point(temp.cols, l), cv::Scalar(0, 0, 255), 2, cv::FILLED );
		}
		
		cv::imshow("Frames", temp);
		
		// waits for awhile depending on the FPS value
		char key = (char)cv::waitKey(1000.0/fFPS);
		
		// checks if ESC was pressed to exit
		if (key == 27) // if 'esc' key is pressed, break loop
		{
			std::cout << "[INFO] Esc key is pressed by user -> Shutting down!" << std::endl;
			return 3; 
		}
		
		// checks if 'g' was pressed to grab the current frames
		if (key == 'g' && foundLR && !bIsCalibrated) 
		{
			std::cout << "[INFO] Checkerboard information is saved: " << iCount << "/" << iNbFrames << std::endl;
				
			// stores 3D and corresponding 2D points
			objpoints.push_back(objp);
			imgpointsL.push_back(cornersL);
			imgpointsR.push_back(cornersR);
			
			// increments the counter
			iCount++;
			
			// creates a blink effect
			bitwise_not(temp, temp);
			cv::imshow("Frames", temp);
			cv::waitKey(200);
		}
		
		// checks if 'u' was pressed to undistort/rectified the frames when calibration is done
		if (key == 'u' && bIsCalibrated) 
		{
			bIsUndistort = !bIsUndistort;
			
			if (bIsUndistort)
				std::cout << "[INFO] Undistort frame"  << std::endl;
			else
				std::cout << "[INFO] Original frame"  << std::endl;
		}
		
		// checks if 's' was pressed to save the matrix intrinsic and distortion information to a yaml file
		if (key == 's' && bIsCalibrated)
		{
			// saves the stereo map in a yaml file
			std::cout << "[INFO] Save the result in a yaml file... ";
			bool isStereoParamsSaved = writeStereoCameraParameters(sStereoParamFilename, camLMatrix, distLCoeffs, camRMatrix, distRCoeffs, Left_Stereo_Map1, Left_Stereo_Map2, Right_Stereo_Map1, Right_Stereo_Map2, Q);
	
			// checks if the stereo camera parameters were successfully read
			if (!isStereoParamsSaved)
			{
				std::cout << std::endl;
				std::cout << "\t[ERROR] Stereo camera parameters could not be saved!" << std::endl;
				return 4;
			}
			std::cout << "OK!" << std::endl;
		}
		
		// performs calibration if number of frames is sufficient
		if (iCount >= iNbFrames && !bIsCalibrated)
		{
			viewL = frameL.clone();
			viewR = frameR.clone();
			cv::hconcat(viewR, viewL, temp);
			cv::putText(temp, "Calibration in progress...", cv::Point(50, 50), 2, 1, cv::Scalar(0, 200, 0), 2);
			cv::imshow("Frames", temp);
			cv::waitKey(200);
			
			// Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat 
			// are calculated. Hence intrinsic parameters are the same.
			cv::Mat Rot, Trns, Emat, Fmat;
			int flag = 0;
			flag |= cv::CALIB_FIX_INTRINSIC;
			
			/*
			* Performing camera calibration by passing the value of known 3D points (objpoints)
			* and corresponding pixel coordinates of the detected corners (imgpoints)
			*/

			// This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
			std::cout << "[INFO] Determine the transformation between the 2 cameras" << std::endl;
			cv::stereoCalibrate(objpoints, imgpointsL, imgpointsR, camLMatrix, distLCoeffs, camRMatrix, distRCoeffs, grayR.size(), Rot, Trns, Emat, Fmat, flag, cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 100, 1e-6)); 
			cv::Mat Rot_vec = cv::Mat::zeros(1,3,CV_64F);
			cv::Rodrigues(Rot, Rot_vec);
			std::cout << "--> Rotation between cameras = " << Rot_vec << std::endl;
			std::cout << "--> Translation between cameras = " << Trns << std::endl;
			
			// Once we know the transformation between the two cameras we can perform stereo rectification
			std::cout << "[INFO] Perform stereo rectification" << std::endl;
			cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r;
			cv::stereoRectify(camLMatrix, distLCoeffs, camRMatrix, distRCoeffs, grayR.size(), Rot, Trns, rect_l, rect_r, proj_mat_l, proj_mat_r, Q, 1);

			// Use the rotation matrices for stereo rectification and camera intrinsics for undistorting the image
			// Compute the rectification map (mapping between the original image pixels and 
			// their transformed values after applying rectification and undistortion) for left and right camera frames
			std::cout << "[INFO] Compute the rectification maps" << std::endl;
			
			cv::initUndistortRectifyMap(camLMatrix, distLCoeffs, rect_l, proj_mat_l, grayL.size(), CV_16SC2, Left_Stereo_Map1, Left_Stereo_Map2);
			cv::initUndistortRectifyMap(camRMatrix, distRCoeffs, rect_r, proj_mat_r, grayR.size(), CV_16SC2, Right_Stereo_Map1, Right_Stereo_Map2);
			
			bIsCalibrated = true;
			
			viewL = frameL.clone();
			viewR = frameR.clone();
			cv::hconcat(viewR, viewL, temp);
			cv::putText(temp, "Calibration done", cv::Point(50, 50), 2, 1, cv::Scalar(0, 200, 0), 2);
			cv::putText(temp, "Press u to visualize undistort frame", cv::Point(50, 100), 2, 1, cv::Scalar(0, 200, 0), 2);
			cv::putText(temp, "Press s to save the intrinsic/distortion params", cv::Point(50, 150), 2, 1, cv::Scalar(0, 200, 0), 2);
			cv::imshow("Frames", temp);
			cv::waitKey(-1);
		}
	}
	
	// releases video streams
	camL.release();
	camR.release();
	
	// destroys all windows
	cv::destroyAllWindows();
	
	return 0;
}
	
	
	
	
	
	
	