#ifndef _UTILS_
#define _UTILS_

#include <opencv2/opencv.hpp>

bool estimateRobotCamTransformation(cv::Mat camDataset, cv::Mat robotDataset, cv::Mat& robotCamTransform);



#endif