#ifndef _IMAGE_PROCESSING_
#define _IMAGE_PROCESSING_

#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//~ #include "DynamixelHandler.h"
//~ #include "Kinematics.h"


// ImageProcessing
#define thresholdValue 200
#define maxValue 255
//ConvertContours
#define drawingAreaWidthInMm 80.0
#define drawingAreaHeigthInMm 50.0
// SendContours
#define deltaTBetweenSamples 500 //in ms
#define incrementBetweenSamples 2

#define DISPARITY_WIN_SIZE 800

std::vector<std::vector<cv::Point>> imageProcessing(cv::Mat originalImage);

std::vector<std::vector<float>> convertContoursPixel2Mm(std::vector<std::vector<cv::Point>> vContoursInPixel, float theta, float tx, float ty, int imageWidth, int imageHeight);

//~ void sendContours(std::vector<std::vector<cv::Point>> vContoursInPixel, std::vector<std::vector<float>> vContoursInMm, DynamixelHandler& dxlHandler,  float L1, float L2, float L3);

void findObject2DPosition(cv::Mat frame, cv::Mat& frameThresholded, bool bIsImageUndistorted, bool isCamParamsSet, cv::Mat cameraMatrix, cv::Mat distCoeffs, int iLowH, int iLowS, int iLowV, int iHighH, int iHighS, int iHighV, int iStructuralElementSize, int iAreaThresold, int& iLastX, int&  iLastY, cv::Mat& imgLines);

void findObject2DPosition(cv::Mat frame, cv::Mat& frameThresholded, int iLowH, int iLowS, int iLowV, int iHighH, int iHighS, int iHighV, int iStructuralElementSize, int iAreaThresold, int& iLastX, int&  iLastY, cv::Mat& imgLines);

void findObject3DPosition(int iLastXR, int iLastYR, int iLastXL, int iLastYL, cv::Mat frameGray, cv::Mat Q, double& x, double& y, double& z);

#endif