#include "Utils.h"


bool estimateRobotCamTransformation(cv::Mat camDataset, cv::Mat robotDataset, cv::Mat& robotCamTransform)
{
	// checks if datasets have the same size
	if (camDataset.dims != robotDataset.dims || camDataset.size != robotDataset.size)
	{
		std::cout << "[ERROR] cam and robot datasets have different sizes!" << std::endl;
		return false;
	}
	
	// computes the mean point of both datasets
	cv::Mat meanCamDataset,  meanRobotDataset;
	cv::reduce(camDataset, meanCamDataset, 1, cv::REDUCE_AVG);
	cv::reduce(robotDataset, meanRobotDataset, 1, cv::REDUCE_AVG);
	//std::cout << "meanCamDataset= " << meanCamDataset << std::endl;
	//std::cout << "meanRobotDataset= " << meanRobotDataset << std::endl;
	
	// normalises the datasets by removing the average
	cv::Mat camDatasetNorm = camDataset;
	cv::Mat robotDatasetNorm = robotDataset;
	cv::Size size = camDataset.size();
	
	for (int x=0; x < size.width; ++x)
	{
		cv::Rect rect(x, 0, 1, size.height);
		camDatasetNorm(rect) = camDataset(rect) - meanCamDataset;
		robotDatasetNorm(rect) = robotDataset(rect) - meanRobotDataset;
	}
	//std::cout << "camDatasetNorm= " << camDatasetNorm << std::endl;
	//std::cout << "robotDatasetNorm= " << robotDatasetNorm << std::endl;
	
	// computes the covariance matrix H
	cv::Mat H;
	H =  camDatasetNorm* robotDatasetNorm.t();
	//std::cout << "H= " << H << std::endl;
	
	// computes the SVD of the covariance matrix
	cv::Mat W, U, Vt;
	cv::SVD::compute(H, W, U, Vt);

	// computes the rotation matrix
	cv::Mat Ut = U.t();
	cv::Mat V = Vt.t();
	cv::Mat R = V * Ut;
	//std::cout << "R= " << R << std::endl;
	
	// forces a right hand coordinate system
	if (determinant(R) < 0)
	{
		for (int l_coord=0; l_coord < 3; l_coord++)
			R.at<double>(l_coord,2) *= -1;
	}
	std::cout << "R= " << R << std::endl;
	cv::Mat Rvec;
	cv::Rodrigues(R, Rvec);
	std::cout << "Rvec= " << Rvec * 57.32<< std::endl;

	// estimates the translation matrix
	cv::Mat t ;
	t = -R * meanCamDataset + meanRobotDataset;
	std::cout << "t= " << t << std::endl;
	
	// fills the matrix with R and t
	R(cv::Range(0,3), cv::Range(0,3)).copyTo(robotCamTransform(cv::Range(0,3), cv::Range(0,3)));
	t(cv::Range(0,3), cv::Range(0,1)).copyTo(robotCamTransform(cv::Range(0,3), cv::Range(3,4)));
	
	return true;
}