#include "detect_sign_on_lane/DetectSignsOnLane.h"

namespace otto_car
{
	namespace lane_markings
	{
	
	DetectSignsOnLane::DetectSignsOnLane(ros::NodeHandle &nh, std::string locationOfDatFile, std::string nameOfRawImageTopic, 
											std::string setDetectionFlagServiceName, bool isDebugMode)
	{
		fullServiceName = ros::this_node::getName() + setDetectionFlagServiceName;
		nameOfInputImageTopic = nameOfRawImageTopic;
		locationOfDataFile = locationOfDatFile;
		nodeHandle = std::shared_ptr<ros::NodeHandle>(&nh);
		debugMode = isDebugMode;
	}

	void DetectSignsOnLane::init()
	{
		setDetectionFlagService = nodeHandle->advertiseService(fullServiceName, &DetectSignsOnLane::setDetectionFlagServiceCallBack, this);
		subscriber = nodeHandle->subscribe(nameOfInputImageTopic, 40, &DetectSignsOnLane::detectSignsFromRawImage, this);
		std::string resultsTopicName = ros::this_node::getName() + nameOfResultsTopic;
		signResultsPublisher = nodeHandle->advertise<detect_sign_on_lane::SignsOnLaneMsg>(resultsTopicName, 50);
		std::string fullDebugResultName = ros::this_node::getName() + nameOfDebugResultsTopic;
		if(debugMode)
		{
			debugImageResultPublisher = nodeHandle->advertise<sensor_msgs::Image>(fullDebugResultName, 50);
		}
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
		cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(image, rosImageToCVEncoding);
		cv::Mat imageFromRaw = imagePtr->image.clone();
		std::vector<SignsOnLaneResult> finalSignResults;

		if(continueDetection && !isDetectionUnderProgress)
		{
			isDetectionUnderProgress = true;
			std::vector<SignsOnLaneResult> orphanNumbers;
			std::vector<std::shared_ptr<cv::RotatedRect>> orphanZebraStripes;
			std::vector<std::shared_ptr<ZebraCrossing>> zebraCrossings;
			std::vector<std::shared_ptr<BarredAreaStripe>> barredAreaStripes;
			std::vector<std::shared_ptr<BarredArea>> barredAreas;

			
			cv::Mat greyImage, thresholdImage;
			cv::cvtColor(imageFromRaw, greyImage, CV_BGR2GRAY);
			cv::threshold(greyImage, thresholdImage, 90, 255, cv::THRESH_BINARY_INV);

			std::vector<std::vector<cv::Point>> contours;
    		std::vector<cv::Vec4i> hierarchy;
			findContours(thresholdImage.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
			for(int i = 0; i < contours.size(); i++)
			{
				SignsOnLaneResult signOnLaneResult;
				std::shared_ptr<BarredAreaStripe> tempBarredAreaStripe;
				if(isContourANumber(imageFromRaw, contours[i], signOnLaneResult))
				{
					orphanNumbers.push_back(signOnLaneResult);
				}
				else if(isContourATurnSign(imageFromRaw, contours[i], signOnLaneResult))
				{
					finalSignResults.push_back(signOnLaneResult);
				}
				else if(isContourASpeedEnd(imageFromRaw, contours[i], signOnLaneResult))
				{
					finalSignResults.push_back(signOnLaneResult);
				}
				else if(ZebraCrossing::isRectZebraStripe(contours[i]))
				{
					std::shared_ptr<cv::RotatedRect> zebraStripe = std::make_shared<cv::RotatedRect>();
					cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);
					zebraStripe->angle = rotatedRect.angle;
					zebraStripe->center.x = rotatedRect.center.x;
					zebraStripe->center.y = rotatedRect.center.y;
					zebraStripe->size.width = rotatedRect.size.width;
					zebraStripe->size.height = rotatedRect.size.height;
					orphanZebraStripes.push_back(zebraStripe);
				}
				else if(BarredAreaStripe::isBarredAreaStripe(contours[i], tempBarredAreaStripe))
				{
					barredAreaStripes.push_back(tempBarredAreaStripe);
				}
				else if(isContourAPedestrianIsland(imageFromRaw, contours[i], signOnLaneResult))
				{
					finalSignResults.push_back(signOnLaneResult);
				}
			}

			mergeOrphanNumbers(orphanNumbers, finalSignResults);

			ZebraCrossing::clusterStripesForZebraCrossings(orphanZebraStripes, zebraCrossings);
			for(int i = 0; i < zebraCrossings.size(); i++)
			{
				if(zebraCrossings[i] != nullptr)
				{
					SignsOnLaneResult signOnLaneResult;
					signOnLaneResult.signType = SIGN_TYPE::ZEBRA_CROSSING;
					signOnLaneResult.value = -1;
					signOnLaneResult.width = zebraCrossings[i]->getWidthOfZebraCrossing();
					signOnLaneResult.height = zebraCrossings[i]->getHeightOfZebraCrossing();
					cv::Point2f centerOfZebraCrossing;
					zebraCrossings[i]->getCenterOfZebraCrossing(centerOfZebraCrossing);
					signOnLaneResult.center = centerOfZebraCrossing;
					std::array<cv::Point2f,4> boundingBox;
					zebraCrossings[i]->boundingBoxOfZebraCrossing(boundingBox);
					for(int i = 0; i < boundingBox.size(); i++)
					{
						signOnLaneResult.boundingBox[i] = boundingBox[i];
					}
					finalSignResults.push_back(signOnLaneResult);
				}
			}

			BarredArea::clusterStripes(barredAreaStripes, barredAreas);
			for(int i = 0; i < barredAreas.size(); i++)
			{
				if(barredAreas[i] != nullptr)
				{
					SignsOnLaneResult signOnLaneResult;
					signOnLaneResult.signType = SIGN_TYPE::BARRED_AREA;
					signOnLaneResult.value = -1;
					signOnLaneResult.width = barredAreas[i]->getWidthOfBarredArea();
					signOnLaneResult.height = barredAreas[i]->getHeightOfBarredArea();
					cv::Point2f centerOfBarredArea;
					barredAreas[i]->getCenter(centerOfBarredArea);
					signOnLaneResult.center = centerOfBarredArea;
					std::array<cv::Point2f,4> boundingBox;
					barredAreas[i]->getBarredAreaBox(boundingBox);
					for(int i = 0; i < boundingBox.size(); i++)
					{
						signOnLaneResult.boundingBox[i] = boundingBox[i];
					}
					finalSignResults.push_back(signOnLaneResult);
				}
			}
			publishResults(finalSignResults);
			isDetectionUnderProgress = false;
		}

		if(debugMode)
		{
			publishResultsForDebug(imageFromRaw, finalSignResults);
		}
	}

	void DetectSignsOnLane::mergeOrphanNumbers(const std::vector<SignsOnLaneResult> &orphanNumbers, std::vector<SignsOnLaneResult> &results)
	{
		std::vector<SignsOnLaneResult> numbers = orphanNumbers;
		while(numbers.size() > 1)
		{
			cv::Point2f number1Center = numbers[numbers.size() - 1].center;
			for(int i = (numbers.size() - 2); i >= 0; i--)
			{
				cv::Point2f number2Center = numbers[i].center;
				float distanceBtwNumbers = BarredAreaStripe::distanceBtwPoints(number1Center, number2Center);
				if((distanceBtwNumbers <= MAX_DISTANCE_SPEED_LIMIT_NUM) && (distanceBtwNumbers >= MIN_DISTANCE_SPEED_LIMIT_NUM))
				{
					SignsOnLaneResult leftNumber, rightNumber, combinedNumber;
					// Determining which number is left most
					if(number1Center.x < number2Center.x)
					{
						leftNumber = numbers[numbers.size() - 1];
						rightNumber = numbers[i];
					}
					else
					{
						leftNumber = numbers[i];
						rightNumber = numbers[numbers.size() - 1];
					}
					combinedNumber.signType = SIGN_TYPE::SPEED_LIMIT_NUMBER;
					combinedNumber.value = (10 * leftNumber.value) + rightNumber.value;
					combinedNumber.height = leftNumber.height;
					combinedNumber.center.x = (leftNumber.center.x + rightNumber.center.x) / 2;
					combinedNumber.center.y = (leftNumber.center.y + rightNumber.center.y) / 2;
					combinedNumber.boundingBox[0] = leftNumber.boundingBox[0];
					combinedNumber.boundingBox[1] = leftNumber.boundingBox[1];
					combinedNumber.boundingBox[2] = rightNumber.boundingBox[2];
					combinedNumber.boundingBox[3] = rightNumber.boundingBox[3];
					combinedNumber.width = BarredAreaStripe::distanceBtwPoints(leftNumber.boundingBox[0], rightNumber.boundingBox[3]);
					results.push_back(combinedNumber);
					numbers.erase(numbers.begin() + i);
					break;
				}
				if(numbers.size() != 0)
				{
					numbers.pop_back();
				}
			}
		}
	}

	void DetectSignsOnLane::publishResultsForDebug(const cv::Mat &originalImage, const std::vector<SignsOnLaneResult> &resultsOnLane)
	{
		for(int i = 0; i < resultsOnLane.size(); i++)
		{
			for(int j = 0; j < 4; j++)
			{
				cv::line(originalImage, resultsOnLane[i].boundingBox[j], resultsOnLane[i].boundingBox[(j+1)%4],
						cv::Scalar(0,255,0));
			}
			cv::Point2f center = resultsOnLane[i].center;
			switch (resultsOnLane[i].signType)
			{
				case END_OF_SPEED_LIMIT:
					cv::putText(originalImage, "Speed limit ends", center, cv::FONT_HERSHEY_DUPLEX, 0.9, CV_RGB(0, 255, 0), 1);
					break;
				case TURN_LEFT:
					cv::putText(originalImage, "Turn left", center, cv::FONT_HERSHEY_DUPLEX, 0.9, CV_RGB(0, 255, 0), 1);
					break;
				case TURN_RIGHT:
					cv::putText(originalImage, "Turn right", center, cv::FONT_HERSHEY_DUPLEX, 0.9, CV_RGB(0, 255, 0), 1);
					break;
				case ZEBRA_CROSSING:
					cv::putText(originalImage, "Zebra crossing", center, cv::FONT_HERSHEY_DUPLEX, 0.9, CV_RGB(0, 255, 0), 1);
					break;
				case BARRED_AREA:
					cv::putText(originalImage, "Barred area", center, cv::FONT_HERSHEY_DUPLEX, 0.9, CV_RGB(0, 255, 0), 1);
					break;
				case PEDESTRIAN_ISLAND:
					cv::putText(originalImage, "Pedestrian island", center, cv::FONT_HERSHEY_DUPLEX, 0.9, CV_RGB(0, 255, 0), 1);
					break;
				case SPEED_LIMIT_NUMBER:
					std::string numberString = std::to_string(resultsOnLane[i].value);
					cv::putText(originalImage, numberString, center, cv::FONT_HERSHEY_DUPLEX, 0.9, CV_RGB(0, 255, 0), 1);
					break;
			}
		}
		cv_bridge::CvImage cvImageMsg;
		cvImageMsg.image = originalImage;
		cvImageMsg.encoding = rosImageToCVEncoding;
		sensor_msgs::Image debugImageMsg;
		cvImageMsg.toImageMsg(debugImageMsg);
		debugImageResultPublisher.publish(debugImageMsg);
	}

	bool DetectSignsOnLane::isContourANumber(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result)
	{
		bool isNumber = false;
		cv::RotatedRect rotatedRect = cv::minAreaRect(contour);
		double angleOfRect = rotatedRect.angle;
		cv::Size2f sizeOfRect = rotatedRect.size;
		double heightOfRect = sizeOfRect.height;
		double widthOfRect = sizeOfRect.width;
		bool isRotatedRectNotInOrder = false;
		if(widthOfRect > heightOfRect)
		{
			widthOfRect = sizeOfRect.height;
			heightOfRect = sizeOfRect.width;
			angleOfRect += 90;
			isRotatedRectNotInOrder = true;
		}

		bool hasDimensionsCloserToNumber = false;
		if((angleOfRect > -60) && (angleOfRect < 60))
		{
            hasDimensionsCloserToNumber = (widthOfRect >= NUMBER_SIGN_RECT_MIN_WIDTH);
			hasDimensionsCloserToNumber = hasDimensionsCloserToNumber && (widthOfRect <= NUMBER_SIGN_RECT_MAX_WIDTH)
             && (heightOfRect >= NUMBER_SIGN_RECT_MIN_HEIGHT)
			 && (heightOfRect <= NUMBER_SIGN_RECT_MAX_HEIGHT);
		}

		if(hasDimensionsCloserToNumber)
		{
			cv::Mat cropAndCompImage;
			cv::Size croppingSize(NUMBER_SIGN_CROPPING_WIDHT, NUMBER_SIGN_CROPPING_HEIGHT);
			cropAndCompressImage(rotatedRect, originalImage, cropAndCompImage, croppingSize);
			int predictionNumber = predictSign(cropAndCompImage);
			if(predictionNumber >= 0 && predictionNumber < 10)
			{
				isNumber = true;
				result.signType = SIGN_TYPE::SPEED_LIMIT_NUMBER;
				result.value = predictionNumber;
				result.width = widthOfRect;
				result.height = heightOfRect;
				result.center = rotatedRect.center;
				rotatedRect.points(result.boundingBox);
				// Making sure that bounding box has points in order ie. bottom left[0], top left[1], top right[2], bottom right[3]
				if(isRotatedRectNotInOrder)
				{
					cv::Point2f tempPoint = result.boundingBox[0];
					result.boundingBox[0] = result.boundingBox[1];
					result.boundingBox[1] = result.boundingBox[2];
					result.boundingBox[2] = result.boundingBox[3];
					result.boundingBox[3] = tempPoint;
				}
			}
		}
		return isNumber;
	}

	bool DetectSignsOnLane::isContourATurnSign(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result)
	{
		bool isTurnSign = false;
		cv::RotatedRect rotatedRect = cv::minAreaRect(contour);
		double angleOfRect = rotatedRect.angle;
		cv::Size2f sizeOfRect = rotatedRect.size;
		double heightOfRect = sizeOfRect.height;
		double widthOfRect = sizeOfRect.width;
		bool isRotatedRectNotInOrder = false;

		if(widthOfRect > heightOfRect)
		{
			widthOfRect = sizeOfRect.height;
			heightOfRect = sizeOfRect.width;
			angleOfRect += 90;
			isRotatedRectNotInOrder = true;
		}

		bool dimensionsCloserToTurnSign = false;
		if((angleOfRect > -60) && (angleOfRect < 60))
		{
            dimensionsCloserToTurnSign = (widthOfRect >= ARROW_SIGN_RECT_MIN_WIDTH);
			dimensionsCloserToTurnSign = dimensionsCloserToTurnSign && (widthOfRect <= ARROW_SIGN_RECT_MAX_WIDTH);
            dimensionsCloserToTurnSign = dimensionsCloserToTurnSign && (heightOfRect >= ARROW_SIGN_RECT_MIN_HEIGHT);
			dimensionsCloserToTurnSign = dimensionsCloserToTurnSign && (heightOfRect <= ARROW_SIGN_RECT_MAX_HEIGHT);
		}

		if(dimensionsCloserToTurnSign)
		{
			cv::Mat cropAndCompImage;
			cv::Size croppingSize(TURN_SIGN_CROPPING_WIDTH, TURN_SIGN_CROPPING_HEIGHT);
			cropAndCompressImage(rotatedRect, originalImage, cropAndCompImage, croppingSize);
			int predictionNumber = predictSign(cropAndCompImage);

			if(predictionNumber == 11)
			{
				isTurnSign = true;
				result.signType = SIGN_TYPE::TURN_LEFT;
				result.value = -1;
				result.width = widthOfRect;
				result.height = heightOfRect;
				result.center = rotatedRect.center;
				rotatedRect.points(result.boundingBox);
				// Making sure that bounding box has points in order ie. bottom left[0], top left[1], top right[2], bottom right[3]
				if(isRotatedRectNotInOrder)
				{
					cv::Point2f tempPoint = result.boundingBox[0];
					result.boundingBox[0] = result.boundingBox[1];
					result.boundingBox[1] = result.boundingBox[2];
					result.boundingBox[2] = result.boundingBox[3];
					result.boundingBox[3] = tempPoint;
				}
			}
			else if(predictionNumber == 12)
			{
				isTurnSign = true;
				result.signType = SIGN_TYPE::TURN_RIGHT;
				result.value = -1;
				result.width = widthOfRect;
				result.height = heightOfRect;
				result.center = rotatedRect.center;
				rotatedRect.points(result.boundingBox);
				// Making sure that bounding box has points in order ie. bottom left[0], top left[1], top right[2], bottom right[3]
				if(isRotatedRectNotInOrder)
				{
					cv::Point2f tempPoint = result.boundingBox[0];
					result.boundingBox[0] = result.boundingBox[1];
					result.boundingBox[1] = result.boundingBox[2];
					result.boundingBox[2] = result.boundingBox[3];
					result.boundingBox[3] = tempPoint;
				}
			}
		}
		return isTurnSign;
	}

	bool DetectSignsOnLane::isContourASpeedEnd(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result)
	{
		bool isSpeedEndSign = false;
		cv::RotatedRect rotatedRect = cv::minAreaRect(contour);
		double angleOfRect = rotatedRect.angle;
		cv::Size2f sizeOfRect = rotatedRect.size;
		double heightOfRect = sizeOfRect.height;
		double widthOfRect = sizeOfRect.width;
		bool isRotatedRectNotInOrder = false;

		if(widthOfRect > heightOfRect)
		{
			widthOfRect = sizeOfRect.height;
			heightOfRect = sizeOfRect.width;
			angleOfRect += 90;
			isRotatedRectNotInOrder = true;
		}

		bool dimensionsCloserToSpeedEnd = false;
		if((angleOfRect > -60) && (angleOfRect < 60))
		{
            dimensionsCloserToSpeedEnd = (widthOfRect >= SPEEDEND_SIGN_RECT_MIN_WIDTH);
			dimensionsCloserToSpeedEnd = dimensionsCloserToSpeedEnd && (widthOfRect <= SPEEDEND_SIGN_RECT_MAX_WIDTH);
            dimensionsCloserToSpeedEnd = dimensionsCloserToSpeedEnd && (heightOfRect >= SPEEDEND_SIGN_RECT_MIN_HEIGHT);
			dimensionsCloserToSpeedEnd = dimensionsCloserToSpeedEnd && (heightOfRect <= SPEEDEND_SIGN_RECT_MAX_HEIGHT);
		}

		if(dimensionsCloserToSpeedEnd)
		{
			cv::Mat cropAndCompImage;
			cv::Size croppingSize(SPEEDEND_CROPPING_WIDTH, SPEEDEND_CROPPING_HEIGHT);
			cropAndCompressImage(rotatedRect, originalImage, cropAndCompImage, croppingSize);
			int predictionNumber = predictSign(cropAndCompImage);

			if(predictionNumber == 10)
			{
				isSpeedEndSign = true;
				result.signType = SIGN_TYPE::END_OF_SPEED_LIMIT;
				result.value = -1;
				result.width = widthOfRect;
				result.height = heightOfRect;
				result.center = rotatedRect.center;
				rotatedRect.points(result.boundingBox);
				// Making sure that bounding box has points in order ie. bottom left[0], top left[1], top right[2], bottom right[3]
				if(isRotatedRectNotInOrder)
				{
					cv::Point2f tempPoint = result.boundingBox[0];
					result.boundingBox[0] = result.boundingBox[1];
					result.boundingBox[1] = result.boundingBox[2];
					result.boundingBox[2] = result.boundingBox[3];
					result.boundingBox[3] = tempPoint;
				}
			}
		}
		return isSpeedEndSign;
	}

	bool DetectSignsOnLane::isContourAPedestrianIsland(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result)
	{
		bool isPedestrianIsland = false;
		cv::RotatedRect rotatedRect = cv::minAreaRect(contour);
		float widthOfRect = rotatedRect.size.width;
		float heightOfRect = rotatedRect.size.height;
		if(widthOfRect > heightOfRect)
		{
			widthOfRect = heightOfRect;
			heightOfRect = rotatedRect.size.width;
		}

		bool isDimensionsCloser = false;
		if(widthOfRect <= PEDESTRIAN_ISLAND_MAX_WIDTH && widthOfRect >= PEDESTRIAN_ISLAND_MIN_WIDTH)
		{
			if(heightOfRect <= PEDESTRIAN_ISLAND_MAX_HEIGHT && heightOfRect >= PEDESTRIAN_ISLAND_MIN_HEIGHT)
			{
				isDimensionsCloser = true;
			}
		}

		if(isDimensionsCloser)
		{
			cv::Mat rotatedImage;
			alignImageToPedestrianIsland(originalImage, rotatedRect, rotatedImage);
			cv::Point2f topCornerOfCroppingBox;
			topCornerOfCroppingBox = rotatedRect.center;
			topCornerOfCroppingBox.x -= (PEDESTRIAN_ISLAND_MIN_WIDTH + PEDESTRIAN_ISLAND_MAX_WIDTH) / 4;
			topCornerOfCroppingBox.x -= 4;
			topCornerOfCroppingBox.x = (topCornerOfCroppingBox.x < 0) ? 0 : topCornerOfCroppingBox.x;
			topCornerOfCroppingBox.y -= (CROPPING_HEIGHT_PEDESTRIAN_ISLAND / 2);
			topCornerOfCroppingBox.y = (topCornerOfCroppingBox.y < 0) ? 0 : topCornerOfCroppingBox.y;
			cv::Rect cropBoxDimensions;
			cropBoxDimensions.width = PEDESTRIAN_ISLAND_MAX_WIDTH + 4;
			cropBoxDimensions.height = CROPPING_HEIGHT_PEDESTRIAN_ISLAND;
			if((topCornerOfCroppingBox.y + cropBoxDimensions.height) > rotatedImage.rows)
			{
				cropBoxDimensions.height = rotatedImage.rows - topCornerOfCroppingBox.y - 1;
			}
			cropBoxDimensions.x = topCornerOfCroppingBox.x;
			cropBoxDimensions.y = topCornerOfCroppingBox.y;
			int predictedNumber = 0;
			try
			{
				cv::Mat croppedImage(rotatedImage, cropBoxDimensions);
				cv::resize(croppedImage, croppedImage, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT));
				cv::threshold(croppedImage, croppedImage, 90, 225, cv::THRESH_BINARY);
				predictedNumber = predictSign(croppedImage);
			}
			catch(const std::exception& e)
			{
				//std::clog << e.what() << '\n';
			}
			
			
			isPedestrianIsland = (predictedNumber == 15);
			if(isPedestrianIsland)
			{
				result.signType = SIGN_TYPE::PEDESTRIAN_ISLAND;
				result.value = -1;
				result.width = (PEDESTRIAN_ISLAND_MIN_WIDTH + PEDESTRIAN_ISLAND_MAX_WIDTH) / 2;
				result.height = (PEDESTRIAN_ISLAND_MIN_HEIGHT + PEDESTRIAN_ISLAND_MAX_HEIGHT) / 2;
				result.center = rotatedRect.center;
				rotatedRect.points(result.boundingBox);
			}
		}
		return isPedestrianIsland;
	}

	void DetectSignsOnLane::alignImageToPedestrianIsland(const cv::Mat &originalImage, const cv::RotatedRect &rotRect, cv::Mat &result)
	{
		cv::Point2f vertices[4];
		rotRect.points(vertices);
		float width = BarredAreaStripe::distanceBtwPoints(vertices[0], vertices[1]);
		float height = BarredAreaStripe::distanceBtwPoints(vertices[1], vertices[2]);
		float slope = 0;
		if(width > height)
		{
			height = width;
			slope = slopeOfLine(vertices[0], vertices[1]);
		}
		else
		{
			slope = slopeOfLine(vertices[1], vertices[2]);
		}
		cv::Mat matRotation;
		float angleOfLine = atan(slope) * 180 / 3.14159;
		float angleToRotate = 0;
		if(angleOfLine <= 0)
		{
			angleToRotate = ((180 + angleOfLine) - 90);
		}
		else
		{
			angleToRotate = -(90 - angleOfLine);
		}
		matRotation = cv::getRotationMatrix2D( rotRect.center, angleToRotate, 1 );
		cv::warpAffine( originalImage, result, matRotation, originalImage.size() );
	}
	
	void DetectSignsOnLane::publishResults(const std::vector<SignsOnLaneResult> &resultsOnLane)
	{
		detect_sign_on_lane::SignsOnLaneMsg finalResults;
		for(int i = 0; i < resultsOnLane.size(); i++)
		{
			detect_sign_on_lane::SignOnLaneMsg singleResult;
			singleResult.signType = (uint8_t)resultsOnLane[i].signType;
			singleResult.value = (int64_t)resultsOnLane[i].value;
			singleResult.width = (float)resultsOnLane[i].width;
			singleResult.height = (float)resultsOnLane[i].height;
			for(int j = 0; j < 4; j++)
			{
				singleResult.boundingBox[j].x = resultsOnLane[i].boundingBox[j].x;
				singleResult.boundingBox[j].y = resultsOnLane[i].boundingBox[j].y;
			}
			finalResults.results.push_back(singleResult);
		}
		if(resultsOnLane.size() > 0)
		{
			signResultsPublisher.publish(finalResults);
		}
	}

	float DetectSignsOnLane::slopeOfLine(const cv::Point2f &point1, const cv::Point2f &point2)
	{
		float result = 0;
		result = (point2.y - point1.y) / (point2.x - point1.x);
		return result;
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

	void DetectSignsOnLane::cropAndCompressImage(const cv::RotatedRect &contourRect, const cv::Mat &originalImage,
														  cv::Mat &resultImage, const cv::Size &cropRectForSize)
	{
		cv::Point2f pointsOfRectCorners[4];
		contourRect.points(pointsOfRectCorners);
		int sizeOfArray = (sizeof(pointsOfRectCorners)/sizeof(*pointsOfRectCorners));
		double multi = 1.3;
		double minX = findMinOrMaxInPoints(pointsOfRectCorners, sizeOfArray, AxisType::X, BoundaryType::MIN);
		double maxX = findMinOrMaxInPoints(pointsOfRectCorners, sizeOfArray, AxisType::X, BoundaryType::MAX);
		double minY = findMinOrMaxInPoints(pointsOfRectCorners, sizeOfArray, AxisType::Y, BoundaryType::MIN);
		double maxY = findMinOrMaxInPoints(pointsOfRectCorners, sizeOfArray, AxisType::Y, BoundaryType::MAX);
		cv::Size size(multi * (maxX - minX), multi * (maxY - minY));
		cv::Mat rotationMatrix2D = cv::getRotationMatrix2D(cv::Point2f(size.width/2, size.height/2), contourRect.angle, 1.0);
		cv::getRectSubPix(originalImage, size, contourRect.center, resultImage);
		cv::warpAffine(resultImage, resultImage, rotationMatrix2D, size);
		cv::getRectSubPix(resultImage, cropRectForSize, cv::Point2f(size.width/2, size.height/2), resultImage);
		cv::resize(resultImage, resultImage, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT));
		cv::cvtColor(resultImage, resultImage, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(resultImage, resultImage, cv::Size(BLUR_IMAGE_WIDTH, BLUR_IMAGE_HEIGHT), 0);
		cv::threshold(resultImage, resultImage, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	}

	int DetectSignsOnLane::predictSign(const cv::Mat &croppedImage)
	{
		std::vector<float> descriptors;
		hog->compute(croppedImage, descriptors);
		int predictionNumber = modelPtr->predict(descriptors);
		return predictionNumber;
	}
	
	}
}