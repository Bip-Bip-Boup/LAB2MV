#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <unistd.h>

#define STEREO_PARAMS_FILENAME "./data/stereo_params.xml"
#define FPS 25.0
#define CAM_LEFT_INDEX 2
#define CAM_RIGHT_INDEX 0
#define RESOLUTION_MAX 800

int main(int argc, char** argv)
{
	// initializes main parameters 
	std::string sStereoParamFilename = STEREO_PARAMS_FILENAME;
	float fFPS = FPS;
	int iCamLIndex = CAM_LEFT_INDEX;
	int iCamRIndex = CAM_RIGHT_INDEX;
			
	// updates main parameters from arguments
	int opt;
	while ((opt = getopt (argc, argv, ":i:f:l:r:")) != -1)
	{
		
		switch (opt)
		{ 
			case 'i':
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
			case '?':
				if (optopt == 'i' || optopt == 'f' || optopt == 'l' || optopt == 'r')
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
	std::cout << "[INFO] Opening stereo calibration params file..";
	cv::FileStorage fsi = cv::FileStorage(sStereoParamFilename, cv::FileStorage::READ);

	// checks if the file storage is opened before continuing
	if (fsi.isOpened() == false)
	{
		std::cout << "[ERROR] Could not open the stereo calibration params file: " <<  sStereoParamFilename << " !"<< std::endl;
		return 3;
	}
	std::cout << "OK!" << std::endl;
	
	// reads the mapping values for stereo image rectification
	cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
	cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;
	cv::Mat Q;
	fsi["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
	fsi["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
	fsi["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
	fsi["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
	fsi["Q"] >> Q;
	
	// releases the reader
	fsi.release();
	
	// creates camera grabbers for the left and right cameras
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
	
	while (true)
	{
		// reads a new frame from left camera
		cv::Mat frameL;
		bool bLCamSuccess = camL.read(frameL);
		
		// reads a new frame from right camera
		cv::Mat frameR;
		bool bRCamSuccess = camR.read(frameR);

		// checks if a new frame was grabbed for both cameras
		if (!bLCamSuccess || !bRCamSuccess) //if not success, break loop
		{
			std::cout << "[WARNING] Could not read a both frames from video streams" << std::endl;
			break;
		}

		// inits matrix for rectified stero images
		cv::Mat Left_nice, Right_nice;

		// applies stereo image rectification on the left image
		cv::remap(frameL, Left_nice, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

		// applies stereo image rectification on the right image
		cv::remap(frameR, Right_nice, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);

		// splits rectified images into channels
		cv::Mat Left_nice_split[3], Right_nice_split[3];
		cv::split(Left_nice, Left_nice_split);
		cv::split(Right_nice, Right_nice_split);
		
		// creates anaglyph image from left/right images channels
		std::vector<cv::Mat> Anaglyph_channels;
			Anaglyph_channels.push_back(Right_nice_split[0]);
			Anaglyph_channels.push_back(Right_nice_split[1]);
			Anaglyph_channels.push_back(Left_nice_split[2]);
		cv::Mat Anaglyph_img;
			cv::merge(Anaglyph_channels, Anaglyph_img);

		// displays the final anaglyph image
		cv::imshow("Anaglyph image", Anaglyph_img);
		
		// waits for awhile depending on the FPS value
		char key = (char)cv::waitKey(1000.0/fFPS);
		// checks if ESC was pressed to exit
		if (key == 27) // if 'esc' key is pressed, break loop
		{
			std::cout << "[INFO] esc key is pressed by user -> Shutting down!" << std::endl;
			return 1; 
		}
	}

	// releases video streams
	camL.release();
	camR.release();
	
	// destroys all windows
	cv::destroyAllWindows();

	return 0;
}
	