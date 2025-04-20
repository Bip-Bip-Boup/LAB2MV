#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"

#include <stdio.h>
#include <iostream>
#include <unistd.h>

#include "ReadWriteFunctions.h"

#define STEREO_PARAMS_FILENAME "./data/stereo_params.xml"
#define DISPARITY_PARAMS_FILENAME "./data/disparity_params.xml"
#define FPS 25.0
#define CAM_LEFT_INDEX 2
#define CAM_RIGHT_INDEX 0
#define RESOLUTION_MAX 800
#define DISPARITY_WIN_SIZE 800

// inits values for StereoBM parameters
	// GUI parameters
int numDisparitiesGUI = 11;  
int blockSizeGUI = 8;
int preFilterTypeGUI = 0;
int preFilterSizeGUI = 16;
int preFilterCapGUI = 34;
int minDisparityGUI = 34;
int textureThresholdGUI = 9; 
int uniquenessRatioGUI = 24;
int speckleRangeGUI = 27;
int speckleWindowSizeGUI = 2;
int disp12MaxDiffGUI = 8;
	// actual parameters
int numDisparities = numDisparitiesGUI*16;  
int blockSize = blockSizeGUI*2+5;
int preFilterType = preFilterTypeGUI;
int preFilterSize = preFilterSizeGUI*2+5;
int preFilterCap = preFilterCapGUI;
int minDisparity = minDisparityGUI;
int textureThreshold = textureThresholdGUI;
int uniquenessRatio = uniquenessRatioGUI;
int speckleRange = speckleRangeGUI;
int speckleWindowSize = speckleWindowSizeGUI * 2;
int disp12MaxDiff = disp12MaxDiffGUI;

// creates an object of StereoBM algorithm
cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();

// creates matrices to store the rectified and undistorted images
cv::Mat niceL, niceR;
cv::Mat disp, disparity;

// defines callback functions for the trackbars to update parameter values
static void on_trackbar1( int, void* ) 
{
	numDisparities = numDisparitiesGUI*16;
	stereo->setNumDisparities(numDisparities);
}

static void on_trackbar2( int, void* )
{
	blockSize = blockSizeGUI*2+5;
	stereo->setBlockSize(blockSize);
}

static void on_trackbar3( int, void* )
{
	preFilterType = preFilterTypeGUI;
	stereo->setPreFilterType(preFilterType);
}

static void on_trackbar4( int, void* )
{
	preFilterSize = preFilterSizeGUI*2+5;
	stereo->setPreFilterSize(preFilterSize);
}

static void on_trackbar5( int, void* )
{
	preFilterCap = preFilterCapGUI;
	stereo->setPreFilterCap(preFilterCap);
}

static void on_trackbar6( int, void* )
{
	textureThreshold = textureThresholdGUI;
	stereo->setTextureThreshold(textureThreshold);
}

static void on_trackbar7( int, void* )
{
	uniquenessRatio = uniquenessRatioGUI;
	stereo->setUniquenessRatio(uniquenessRatio);
}

static void on_trackbar8( int, void* )
{
	speckleRange = speckleRangeGUI;
	stereo->setSpeckleRange(speckleRange);
}

static void on_trackbar9( int, void* )
{
	speckleWindowSize = speckleWindowSizeGUI * 2;
	stereo->setSpeckleWindowSize(speckleWindowSize);
}

static void on_trackbar10( int, void* )
{
	disp12MaxDiff = disp12MaxDiffGUI;
	stereo->setDisp12MaxDiff(disp12MaxDiff);
}

static void on_trackbar11( int, void* )
{
	minDisparity = minDisparityGUI;
	stereo->setMinDisparity(minDisparity);
}


int main(int argc, char** argv)
{
	// initializes main parameters 
	std::string sStereoParamFilename = STEREO_PARAMS_FILENAME;
	std::string sDisparityParamFilename = DISPARITY_PARAMS_FILENAME;
	float fFPS = FPS;
	int iCamLIndex = CAM_LEFT_INDEX;
	int iCamRIndex = CAM_RIGHT_INDEX;
	
	// updates main parameters from arguments
	int opt;
	while ((opt = getopt (argc, argv, ":i:o:f:l:r:")) != -1)
	{
		switch (opt)
		{
			case 'i':
				sStereoParamFilename = optarg;
				break;
			case 'o':
				sDisparityParamFilename = optarg;
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
			case '?':
				if (optopt == 'i' || optopt == 'o' || optopt == 'f' || optopt == 'l' || optopt == 'r')
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
	
	// creates a reader to retrieve the stereo calibration params
	std::cout << "[INFO] Opening stereo calibration params file...";
	cv::Mat camLMatrix, distLCoeffs, camRMatrix, distRCoeffs, stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y, Q;
	bool isStereoParamsLoaded = readStereoCameraParameters(sStereoParamFilename, camLMatrix, distLCoeffs, camRMatrix, distRCoeffs, stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y, Q);
	
	// checks if the file storage is opened before continuing
	if (!isStereoParamsLoaded)
	{
		std::cout << "[ERROR] Could not open the stereo calibration params file: " <<  sStereoParamFilename << " !"<< std::endl;
		return 3;
	}
	std::cout << "OK!" << std::endl;
	
	// creates camera grabbers for the left and right cameras
	cv::VideoCapture camL(iCamLIndex, cv::CAP_V4L2), camR(iCamRIndex, cv::CAP_V4L2);
	std::cout << "[INFO] Opening camera videostreams... ";
	
	// changes image resolution to maximum i.e. 1920x1080 if possible
	camL.set(cv::CAP_PROP_FRAME_HEIGHT, RESOLUTION_MAX); camL.set(cv::CAP_PROP_FRAME_WIDTH, RESOLUTION_MAX);
	camR.set(cv::CAP_PROP_FRAME_HEIGHT, RESOLUTION_MAX); camR.set(cv::CAP_PROP_FRAME_WIDTH, RESOLUTION_MAX);
	
	// checks if the left camera was successfully opened
	if (!camL.isOpened()) // if not success, exit program
	{
		std::cout << std::endl;
		std::cout << "\t[ERROR] Could not open the left camera!" << std::endl;
		return 1;
	}

	// checks if the right camera was successfully opened
	if (!camR.isOpened()) // if not success, exit program
	{
		std::cout << std::endl;
		std::cout << "\t[ERROR] Could not open the right camera!" << std::endl;
		return 2;
	}
	std::cout << "OK!" << std::endl;

	// gets image resolution for info
	std::cout << "[INFO] Left camera resolution: " << camL.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camL.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << "[INFO] Right camera resolution: " << camR.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << camR.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	
	// creates a named window that can be linked to the trackbars and displays disparity
	cv::namedWindow("disparity", cv::WINDOW_NORMAL);
	//cv::namedWindow("disparity_norm", cv::WINDOW_NORMAL);
	cv::resizeWindow("disparity", DISPARITY_WIN_SIZE, DISPARITY_WIN_SIZE);
		
	// creates trackbars to dynamically update the StereoBM parameters
	cv::createTrackbar("numDisparities", "disparity", &numDisparitiesGUI, 20, on_trackbar1);
	cv::createTrackbar("blockSize", "disparity", &blockSizeGUI, 50, on_trackbar2);
	cv::createTrackbar("preFilterType", "disparity", &preFilterTypeGUI, 1, on_trackbar3);
	cv::createTrackbar("preFilterSize", "disparity", &preFilterSizeGUI, 25, on_trackbar4);
	cv::createTrackbar("preFilterCap", "disparity", &preFilterCapGUI, 62, on_trackbar5);
	cv::createTrackbar("textureThreshold", "disparity", &textureThresholdGUI, 100, on_trackbar6);
	cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatioGUI, 100, on_trackbar7);
	cv::createTrackbar("speckleRange", "disparity", &speckleRangeGUI, 100, on_trackbar8);
	cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);
	cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiffGUI, 25, on_trackbar10);
	cv::createTrackbar("minDisparity", "disparity", &minDisparityGUI, 50, on_trackbar11);

	// inits display
	cv::namedWindow("Frames", cv::WINDOW_NORMAL);
	cv::resizeWindow("Frames", 1280, 480);
	
	// creates mat file to store images
	cv::Mat frameL, frameR;
	cv::Mat frameL_gray, frameR_gray;
	
	while (true)
	{
		// reads a new frame from left/right cameras
		bool bLCamSuccess = camL.read(frameL);
		bool bRCamSuccess = camR.read(frameR);

		// checks if a new frame was grabbed for both cameras
		if (!bLCamSuccess || !bRCamSuccess) //if not success, break loop
		{
			std::cout << "[WARNING] Could not read a both frames from video streams" << std::endl;
			break;
		}
		
		// converts images to grayscale
		cv::cvtColor(frameL, frameL_gray, cv::COLOR_BGR2GRAY);
		cv::cvtColor(frameR, frameR_gray, cv::COLOR_BGR2GRAY);

		// applies stereo image rectification on the left/right images
		cv::remap(frameL_gray, niceL, stereoMapL_x, stereoMapL_y, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
		cv::remap(frameR_gray, niceR, stereoMapR_x, stereoMapR_y, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

		// concats left & right nice frames and displays it
		cv::Mat view;
		cv::hconcat(niceR, niceL, view);
		
		for (int l=0; l< frameL.rows; l+=100) 
			cv::line(view, cv::Point(0,l), cv::Point(view.cols, l), cv::Scalar(0, 0, 255), 2, cv::FILLED );
		
		cv::imshow("Frames", view);

		// resizes left and right rectified images to be squared (works better with SGBM algo
		cv::resize(niceL, niceL,cv::Size(DISPARITY_WIN_SIZE, DISPARITY_WIN_SIZE));
		cv::resize(niceR, niceR,cv::Size(DISPARITY_WIN_SIZE, DISPARITY_WIN_SIZE));
		
		// calculates disparity using the StereoBM algorithm
		if (numDisparities!= 0)
			stereo->compute(niceL, niceR, disp);
		
		// normalizes and displays disparity
		cv::normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);  
		//cv::applyColorMap(disp, disp, cv::COLORMAP_JET);
		cv::imshow("disparity",disp);
		// converts disparity to "real" values
		/*disp.convertTo(disparity, CV_32F, 1.0);
		disparity = ((disparity/16.0f - (float)minDisparity) / ((float)numDisparities))*255.0;
		disparity.convertTo(disparity, CV_8U);
		cv::applyColorMap(disparity, disparity, cv::COLORMAP_JET);
		
		// displays it using JET colormap		
		cv::imshow("disparity_norm",disparity);*/
		
		
		// NOTE: compute returns a 16bit signed single channel image,
		// CV_16S containing a disparity map scaled by 16. Hence it 
		// is essential to convert it to CV_32F and scale it down 16 times.
		// converts disparity values to CV_32F from CV_16S
		
  
		// waits for awhile depending on the FPS value
		char key = (char)cv::waitKey(1000.0/fFPS);
		
		// checks if ESC was pressed to exit
		if (key == 27) // if 'esc' key is pressed, break loop
		{
			std::cout << "[INFO] esc key is pressed by user -> Shutting down!" << std::endl;
			break; 
		}
		
		// checks if 's' was pressed to save the disparity params to a yaml file
		if (key == 's' )
		{
			// saves the disparity params in a yaml file
			std::cout << "[INFO] Save the result in a yaml file... ";
			bool isDisparityParamsSaved = writeDisparityParameters(sDisparityParamFilename, numDisparities, blockSize , preFilterType, preFilterSize, preFilterCap,  textureThreshold, uniquenessRatio, speckleRange, speckleWindowSize, disp12MaxDiff, minDisparity);
	
			// checks if the disparity parameters were successfully read
			if (!isDisparityParamsSaved)
			{
				std::cout << std::endl;
				std::cout << "\t[ERROR] Disparity parameters could not be saved!" << std::endl;
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
