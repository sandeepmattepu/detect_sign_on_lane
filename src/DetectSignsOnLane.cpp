#include "detect_sign_on_lane/DetectSignsOnLane.h"

namespace otto_car
{
	namespace lane_markings
	{
	
	DetectSignsOnLane::DetectSignsOnLane(ros::NodeHandle &nh, std::string nameOfLaneImageTopic)
	{
		std::string fullServiceName = ros::this_node::getName() + this->setDetectionFlagServiceName;
		this->setDetectionFlagService = nh.advertiseService(fullServiceName, &DetectSignsOnLane::setDetectionFlagServiceCallBack, this);
		
		nh.subscribe(nameOfLaneImageTopic, 40, &DetectSignsOnLane::detectSignsFromRawImage, this);

		this->constructHogObject();
	}

	bool DetectSignsOnLane::setDetectionFlagServiceCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
	{
		this->continueDetection = req.data;

		res.success = this->continueDetection;
		if(res.success)
		{
			res.message = "Detection of signs on the lane has started";
			ROS_INFO("Detection of signs on the lane has started");
		}
		else
		{
			res.message = "Detection of signs on the lane is paused/stopped";
			ROS_INFO("Detection of signs on the lane is paused/stopped");
		}
		return true;
	}

	void DetectSignsOnLane::detectSignsFromRawImage(const sensor_msgs::Image &image)
	{
		if(this->continueDetection)
		{
			cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(image, rosImageToCVEncoding);
			imageFromRaw = imagePtr->image.clone();
			cv::cvtColor(imageFromRaw, greyImage, CV_BGR2GRAY);
			cv::threshold(greyImage, thresholdImage, 90, 255, cv::THRESH_BINARY_INV);

			findContours(thresholdImage.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
			for(int i = 0; i < contours.size(); i++)
			{
				isContourSign = filterContorsForSigns(contours[i]);
				if(isContourSign)
				{

				}
			}
		}
	}

	cv::HOGDescriptor DetectSignsOnLane::constructHogObject()
	{
		cv::Size winSize(20,20);
        cv::Size blockSize(8,8);
        cv::Size blockStride(4,4);
        cv::Size cellSize(8,8);
        int nbins = 9;
        int derivAperture = 1;
        double winSigma = -1.;
        int histogramNormType = 0;
        double L2HysThreshold = 0.2;
        bool gammaCorrection = true;
        int nlevels = 64;
        bool signedGradient = true;
        this->hog = cv::HOGDescriptor(winSize, blockSize, blockStride, cellSize, nbins, derivAperture, winSigma, histogramNormType,
        				L2HysThreshold, gammaCorrection, nlevels, signedGradient);
	}

	bool DetectSignsOnLane::filterContorsForSigns(std::vector<cv::Point> contour)
	{
		rotatedRect = cv::minAreaRect(contour);
		angleOfRect = rotatedRect.angle;
		sizeOfRect = rotatedRect.size;
		heightOfRect = sizeOfRect.height;
		widthOfRect = sizeOfRect.width;

		if(widthOfRect > heightOfRect)
		{
			angleOfRect += 90;
		}

		if((angleOfRect > -60) && (angleOfRect < 60))
		{
            isNumber = (widthOfRect >= NUMBER_SIGN_RECT_MIN_WIDTH) && (widthOfRect < NUMBER_SIGN_RECT_MAX_WIDTH);
            isNumber = isNumber && ((heightOfRect >= NUMBER_SIGN_RECT_MIN_HEIGHT) && (heightOfRect < NUMBER_SIGN_RECT_MAX_HEIGHT));

            isArrow = (widthOfRect >= ARROW_SIGN_RECT_MIN_WIDTH) && (widthOfRect < ARROW_SIGN_RECT_MAX_WIDTH);
            isArrow = isArrow && ((heightOfRect >= ARROW_SIGN_RECT_MIN_HEIGHT) && (heightOfRect < ARROW_SIGN_RECT_MAX_HEIGHT));

            isSpeedEnd = (widthOfRect >= SPEEDEND_SIGN_RECT_MIN_WIDTH) && (widthOfRect < SPEEDEND_SIGN_RECT_MAX_WIDTH);
            isSpeedEnd = isSpeedEnd && ((heightOfRect >= SPEEDEND_SIGN_RECT_MIN_HEIGHT) && (heightOfRect < SPEEDEND_SIGN_RECT_MAX_HEIGHT));

            return (isNumber || isArrow || isSpeedEnd);
		}

		return false;
	}
	
	}
}