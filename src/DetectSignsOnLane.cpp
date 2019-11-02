#include "detect_sign_on_lane/DetectSignsOnLane.h"

namespace otto_car
{
	namespace lane_markings
	{
	
	DetectSignsOnLane::DetectSignsOnLane(ros::NodeHandle &nh, std::string locationOfDatFile, std::string nameOfRawImageTopic, 
											std::string setDetectionFlagServiceName)
	{
		fullServiceName = ros::this_node::getName() + setDetectionFlagServiceName;
		nameOfInputImageTopic = nameOfRawImageTopic;
		locationOfDataFile = locationOfDatFile;
		nodeHandle = std::shared_ptr<ros::NodeHandle>(&nh);
		tempCVImage = std::make_shared<cv_bridge::CvImage>();
	}

	void DetectSignsOnLane::init()
	{
		setDetectionFlagService = nodeHandle->advertiseService(fullServiceName, &DetectSignsOnLane::setDetectionFlagServiceCallBack, this);
		subscriber = nodeHandle->subscribe(nameOfInputImageTopic, 40, &DetectSignsOnLane::detectSignsFromRawImage, this);
		std::string fullDebugResultName = ros::this_node::getName() + "/debug_result_image";
		debugImageResultPublisher = nodeHandle->advertise<sensor_msgs::Image>(fullDebugResultName, 50);
		constructHogObject();
		modelPtr = cv::ml::SVM::load(locationOfDataFile);
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
		//if(continueDetection)
		//{
			cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(image, rosImageToCVEncoding);
			imageFromRaw = imagePtr->image.clone();
			croppedImage = imageFromRaw.clone();
			cv::cvtColor(imageFromRaw, greyImage, CV_BGR2GRAY);
			cv::threshold(greyImage, thresholdImage, 90, 255, cv::THRESH_BINARY_INV);

			findContours(thresholdImage.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
			for(int i = 0; i < contours.size(); i++)
			{
				rotatedRect = cv::minAreaRect(contours[i]);
				isContourASign = filterContorsForSigns(rotatedRect, cropRect);
				if(isContourASign)
				{
					preprocessBeforeSignDetection(rotatedRect, imageFromRaw, croppedImage, cropRect);
					predictSign(croppedImage, contours[i], rotatedRect);
				}
			}
		//}
			
		tempCVImage->image = croppedImage;
		tempCVImage->encoding = "bgr8";
		tempCVImage->toImageMsg(debugImage);
		debugImageResultPublisher.publish(debugImage);
	}

	void DetectSignsOnLane::constructHogObject()
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
        hog = new cv::HOGDescriptor(winSize, blockSize, blockStride, cellSize, nbins, derivAperture, winSigma, histogramNormType,
        				L2HysThreshold, gammaCorrection, nlevels, signedGradient);
	}

	bool DetectSignsOnLane::filterContorsForSigns(cv::RotatedRect &contourRect, cv::Size &cropRectForSize)
	{
		angleOfRect = contourRect.angle;
		sizeOfRect = contourRect.size;
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

            if(isSpeedEnd)
            {
            	cropRectForSize = cv::Size(SPEEDEND_CROP_RECT_WIDTH, SPEEDEND_CROP_RECT_HEIGHT);
            }
            else
            {
            	cropRectForSize = cv::Size(OTHERSIGN_CROP_RECT_WIDTH, OTHERSIGN_CROP_RECT_HEIGHT);
            }

            return (isNumber || isArrow || isSpeedEnd);
		}

		return false;
	}

	float DetectSignsOnLane::findMinOrMaxInPoints(cv::Point2f points[], int sizeOfArray, AxisType axis, BoundaryType boundary)
	{
		float result = 0;
		for(int i = 0; i < sizeOfArray; i++)
		{
			if(i == 0)
			{
				result = (axis == AxisType::X) ? points[i].x : points[i].y;
				continue;
			}
			else
			{
				float toCompareWith = (axis == AxisType::X) ? points[i].x : points[i].y;
				if(boundary == BoundaryType::MIN)
				{
					result = (toCompareWith < result) ? toCompareWith : result;
				}
				else
				{
					result = (toCompareWith > result) ? toCompareWith : result;
				}
			}
		}
		return result;
	}

	void DetectSignsOnLane::preprocessBeforeSignDetection(cv::RotatedRect &contourRect, cv::Mat &image, cv::Mat &result,
															 cv::Size &cropRectForSize)
	{
		contourRect.points(pointsOfRectCorners);
		int sizeOfArray = (sizeof(pointsOfRectCorners)/sizeof(*pointsOfRectCorners));
		minX = findMinOrMaxInPoints(pointsOfRectCorners, sizeOfArray, AxisType::X, BoundaryType::MIN);
		maxX = findMinOrMaxInPoints(pointsOfRectCorners, sizeOfArray, AxisType::X, BoundaryType::MAX);
		minY = findMinOrMaxInPoints(pointsOfRectCorners, sizeOfArray, AxisType::Y, BoundaryType::MIN);
		maxY = findMinOrMaxInPoints(pointsOfRectCorners, sizeOfArray, AxisType::Y, BoundaryType::MAX);
		int width = (multi * (maxX - minX));
		int height = (multi * (maxY - minY));
		size = cv::Size(multi * (maxX - minX), multi * (maxY - minY));
		rotationMatrix2D = cv::getRotationMatrix2D(cv::Point2f(size.width/2, size.height/2), contourRect.angle, 1.0);
		cv::getRectSubPix(image, size, contourRect.center, result);
		cv::warpAffine(result, result, rotationMatrix2D, size);
		cv::getRectSubPix(result, cropRectForSize, cv::Point2f(size.width/2, size.height/2), result);
		cv::resize(result, result, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT));
		cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(result, result, cv::Size(BLUR_IMAGE_WIDTH, BLUR_IMAGE_HEIGHT), 0);
		cv::threshold(result, result, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	}

	void DetectSignsOnLane::predictSign(cv::Mat &image, std::vector<cv::Point> &contour, cv::RotatedRect &contourRect)
	{
		contoursToDraw.clear();
		contoursToDraw.push_back(contour);
		hog->compute(image, descriptors);
		predictionNumber = modelPtr->predict(descriptors);

		if(predictionNumber >=0 && predictionNumber < 14)
		{
			cv::drawContours(image, contoursToDraw, 0, cv::Scalar(0,0,225), 2);
		}

		if(predictionNumber >= 0 && predictionNumber < 10)
		{
			cv::putText(image, std::to_string(predictionNumber), contourRect.center, cv::FONT_HERSHEY_DUPLEX, 0.9, 
							CV_RGB(0, 255, 0), 1);
		}
		else if(predictionNumber >= 10 && predictionNumber < 14)
		{
			if(predictionNumber == 11)
			{
				labelString = "Turn left";
			}
			else if(predictionNumber == 12)
			{
				labelString = "Turn right";
			}
			else
			{
				labelString = "Speed limit ends";
			}
			cv::putText(image, labelString, contourRect.center, cv::FONT_HERSHEY_DUPLEX, 0.9, 
							CV_RGB(0, 255, 0), 1);
		}
		descriptors.clear();
	}
	
	}
}