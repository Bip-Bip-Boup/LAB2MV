#include "ImageProcessing.h"

std::vector<std::vector<cv::Point>> imageProcessing(cv::Mat img_original)
{
	// displays the original image
	cv::imshow( "Original", img_original );
	
	// converts the image to grayscale
	cv::Mat img_gray;
	cv::cvtColor( img_original, img_gray, cv::COLOR_BGR2GRAY );
	
	// displays the grayscale image
	cv::imshow( "Grayscale", img_gray );
	
	cv::Mat img_threshold;
	cv::threshold(img_gray, img_threshold, thresholdValue, maxValue, cv::THRESH_BINARY);
	
	if (false)
	{
		// blurs it
		cv::Mat img_blur;
		blur( img_gray, img_blur, cv::Size(3,3) );
		
		// displays the blurred image
		cv::imshow( "Blur", img_blur );
		
		// applies Canny algorithm on it
		cv::Mat canny_output;
		cv::Canny( img_blur, canny_output, thresholdValue, maxValue );
		
		// displays the canny image
		cv::imshow( "Canny", canny_output );
	}
	
	// finds contours on the canny image
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( img_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE );
	//cv::findContours( canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE );
	
	// displays contours on the original image
	cv::RNG rng(12345);
	//cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
	cv::Mat drawing = cv::Mat::zeros( img_threshold.size(), CV_8UC3 );
	for( size_t i = 0; i< contours.size(); i++ )
	{
		cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
		cv::drawContours( drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
	}
	cv::imshow( "Contours", drawing );
	
	return contours;
}

std::vector<std::vector<float>> convertContoursPixel2Mm(std::vector<std::vector<cv::Point>> vContoursInPixel, float theta, float tx, float ty, int imageWidth, int imageHeight)
{
	std::vector<std::vector<float>> vContoursInMm;
	
	// pixel / mm ratios
	float widthRatio = (float)imageWidth / drawingAreaWidthInMm;
	float heigthRatio = (float)imageHeight / drawingAreaHeigthInMm;
	float appliedRatio = ceil((widthRatio>=heigthRatio) ? widthRatio : heigthRatio);
	
	// image to real world reference frame transformation
	cv::Mat transformationMatrix(3, 3, CV_32FC1);
	transformationMatrix.at<float>(0,0) = cos(theta); 	transformationMatrix.at<float>(0,1) = -sin(theta);	transformationMatrix.at<float>(0,2) = tx;
	transformationMatrix.at<float>(1,0) = sin(theta); 	transformationMatrix.at<float>(1,1) = cos(theta);	transformationMatrix.at<float>(1,2) = ty;
	transformationMatrix.at<float>(2,0) = 0.0;		transformationMatrix.at<float>(2,1) = 0.0;		transformationMatrix.at<float>(2,2) = 1.0;
	
	// size the output vector with the same number of contours as the input one
	vContoursInMm.resize(vContoursInPixel.size());
	
	// fill the output vector by transforming the data
	for (int l_contour = 0; l_contour < vContoursInPixel.size(); l_contour++)
	{
		std::cout << "[INFO] (convertContoursPixel2Mm) contour #" << l_contour << " , # of pts = " << vContoursInPixel[l_contour].size()<< std::endl;
		for (int l_point = 0; l_point < vContoursInPixel[l_contour].size(); l_point++)
		{
			// get the coordinate of the current point in the pixel reference frame
			cv::Mat pointCoordinateInPixel(3, 1, CV_32FC1);
			pointCoordinateInPixel.at<float>(0,0) = vContoursInPixel[l_contour][l_point].y/ appliedRatio; //!! WATCH OUT Points go (x,y); (width,height)
			pointCoordinateInPixel.at<float>(1,0) = vContoursInPixel[l_contour][l_point].x/ appliedRatio;
			pointCoordinateInPixel.at<float>(2,0) = 1.0;
			
			// transform it in the real world reference frame
			cv::Mat pointCoordinateInMm(3, 1, CV_32FC1);
			pointCoordinateInMm = transformationMatrix * pointCoordinateInPixel;
			
			// store the data in the contour in mm
			vContoursInMm[l_contour].push_back(pointCoordinateInMm.at<float>(0,0) );
			vContoursInMm[l_contour].push_back(pointCoordinateInMm.at<float>(1,0) );
		}
	}
	
	return vContoursInMm;
	
}

//~ void sendContours(std::vector<std::vector<cv::Point>> vContoursInPixel, std::vector<std::vector<float>> vContoursInMm, DynamixelHandler& dxlHandler, float L1, float L2, float L3)
//~ {
	//~ if (vContoursInPixel.size() == 0 || vContoursInMm.size() == 0)
	//~ {
		//~ std::cout << "[ERROR](sendContours) Contour vector is empty!" << std::endl;
		//~ return;
	//~ }
	
	//~ cv::Mat drawing = cv::Mat::zeros( 240, 240, CV_8UC3 );
	//~ cv::Scalar color = cv::Scalar( 0, 0, 255 );
	//~ cv::imshow( "Current contour", drawing );
	
	//~ float q_pen = deg2rad(-70.0f);
	
	//~ for (int l_contour = 0; l_contour < vContoursInMm.size(); l_contour++)
	//~ {
		//~ q_pen = deg2rad(-70.0f);
		//~ std::cout << "[INFO] (sendContours) contour #" << l_contour << " , # of pts = " << vContoursInMm[l_contour].size()<< std::endl;
		//~ for (int l_point = 0; l_point < vContoursInMm[l_contour].size()/2; l_point=l_point+incrementBetweenSamples)
		//~ {
			//~ std::cout << "l_point #" << l_point << std::endl;
			//~ float targetPointX = vContoursInMm[l_contour][2*l_point]/10; // in cm
			//~ float targetPointY = vContoursInMm[l_contour][2*l_point+1]/10; // in cm
			//~ // TODO
			//~ float targetPointZ;
			
			//~ float targetPointI = vContoursInPixel[l_contour][l_point].x;
			//~ float targetPointJ = vContoursInPixel[l_contour][l_point].y;
			
			//~ // Plots a dot at the current point position
			//~ cv::circle(drawing, cv::Point(targetPointI ,targetPointJ), 0, cv::Scalar(0,0,255), cv::FILLED, cv::LINE_8);
			//~ cv::imshow( "Current contour", drawing );
			//~ std::cout<<"CurrentPoint = (" << targetPointJ << "," << targetPointI << ") --> (" << targetPointX << "," << targetPointY << ")" << std::endl;
			
			//~ // IK
			//~ std::vector<float> qi = computeInverseKinematics(targetPointX, targetPointY, targetPointZ, L1, L2, L3);
			//~ if (qi.size() > 3)
			//~ {
				//~ std::vector<float> l_vTargetJointPosition;
				//~ // q1
				//~ l_vTargetJointPosition.push_back(qi[1]);
				//~ // fixed joint
				//~ l_vTargetJointPosition.push_back(q_pen);
				//~ // q2
				//~ l_vTargetJointPosition.push_back(qi[2]);
				
				//~ dxlHandler.sendTargetJointPosition(l_vTargetJointPosition);
			//~ }
			//~ q_pen = deg2rad(-90.0f);
			//~ cv::waitKey(deltaTBetweenSamples);
		//~ }
	//~ }
//~ }

void findObject2DPosition(cv::Mat frame, cv::Mat& frameThresholded, bool bIsImageUndistorted, bool isCamParamsSet, cv::Mat cameraMatrix, cv::Mat distCoeffs, int iLowH, int iLowS, int iLowV, int iHighH, int iHighS, int iHighV, int iStructuralElementSize, int iAreaThresold, int& iLastX, int&  iLastY, cv::Mat& imgLines)
{	
	
	// undistorts images for both cameras
	if (bIsImageUndistorted && isCamParamsSet)
	{
		cv::Mat temp = frame.clone();
		cv::undistort(temp, frame, cameraMatrix, distCoeffs);
	}
	
	//Convert the captured frame from BGR to HSV
	cv::Mat frameHSV;
	cvtColor(frame, frameHSV, cv::COLOR_BGR2HSV); 
	
	//Threshold the image based on the trackbar values
	inRange(frameHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), frameThresholded); 
	
	//morphological opening (remove small objects from the foreground)
	cv::erode(frameThresholded, frameThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) );
	cv::dilate( frameThresholded, frameThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) ); 
	
	//morphological closing (fill small holes in the foreground)
	cv::dilate( frameThresholded, frameThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) ); 
	cv::erode(frameThresholded, frameThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) );
	
	// calculates the moments of the thresholded image
	cv::Moments oMoments = moments(frameThresholded);
	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	// if the area <= iAreaThresold, considers that the there are no object in the image and it's because of the noise, the area is not zero 
	if (dArea > iAreaThresold)
	{
		// calculates the position of the ball
		int posX = dM10 / dArea;
		int posY = dM01 / dArea;        

		if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
		{
			// draw a red line from the previous point to the current point
			line(imgLines, cv::Point(posX, posY), cv::Point(iLastX, iLastY), cv::Scalar(0,0,255), 2);
		}

		// stores the current position for enxt frame
		iLastX = posX;
		iLastY = posY;
	}
	
}

void findObject2DPosition(cv::Mat frame, cv::Mat& frameThresholded, int iLowH, int iLowS, int iLowV, int iHighH, int iHighS, int iHighV, int iStructuralElementSize, int iAreaThresold, int& iLastX, int&  iLastY, cv::Mat& imgLines)
{	
	//Convert the captured frame from BGR to HSV
	cv::Mat frameHSV;
	cvtColor(frame, frameHSV, cv::COLOR_BGR2HSV); 
	
	//Threshold the image based on the trackbar values
	inRange(frameHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), frameThresholded); 
	
	//morphological opening (remove small objects from the foreground)
	cv::erode(frameThresholded, frameThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) );
	cv::dilate( frameThresholded, frameThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) ); 
	
	//morphological closing (fill small holes in the foreground)
	cv::dilate( frameThresholded, frameThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) ); 
	cv::erode(frameThresholded, frameThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iStructuralElementSize, iStructuralElementSize)) );
	
	// calculates the moments of the thresholded image
	cv::Moments oMoments = moments(frameThresholded);
	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	// if the area <= iAreaThresold, considers that the there are no object in the image and it's because of the noise, the area is not zero 
	if (dArea > iAreaThresold)
	{
		// calculates the position of the ball
		int posX = dM10 / dArea;
		int posY = dM01 / dArea;        

		if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
		{
			// draw a red line from the previous point to the current point
			line(imgLines, cv::Point(posX, posY), cv::Point(iLastX, iLastY), cv::Scalar(0,0,255), 2);
		}

		// stores the current position for enxt frame
		iLastX = posX;
		iLastY = posY;
	}
	
}

void findObject3DPosition(int iLastXR, int iLastYR, int iLastXL, int iLastYL, cv::Mat frameThresholded, cv::Mat Q, double& x, double& y, double& z)
{
	// estimates average position between the left and right images
	float v = (float)iLastYL;
	float u = (float)iLastXL;
	float disparity = abs((float)iLastXR - (float)iLastXL);
	std::cout << "disp= " << disparity << std::endl;
		
	// projects 2D position to 3D world
	cv::Mat_<double> vec_tmp(4,1);
	if (u > 0 & u < frameThresholded.cols & v > 0 & v < frameThresholded.rows)
	{
		vec_tmp(0)=(double)u; 
		vec_tmp(1)=(double)v;
		vec_tmp(2)=(double)disparity;
		vec_tmp(3)=1.0;
		
		vec_tmp = Q*vec_tmp;
		vec_tmp /= vec_tmp(3);
	}
	
	// saves cartesian coordinates
	x = vec_tmp(0);
	y = vec_tmp(1);
	z = vec_tmp(2);
}
