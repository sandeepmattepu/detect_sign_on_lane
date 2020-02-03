#ifndef DETECT_SIGNS_ON_LANE
#define DETECT_SIGNS_ON_LANE

#include <string>
#include <array>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/ml.hpp"

#include "ZebraCrossing.h"
#include "BarredArea.h"
#include "SignsOnLaneResult.h"

namespace otto_car
{
	namespace lane_markings
	{

		enum AxisType { X, Y};
		enum BoundaryType { MIN, MAX};

		/**
		* This class is responsible for detecting the code 
		*/
		class DetectSignsOnLane
		{
			private:
				/* The following constrain values are obtained by testing. If you alter the camera configuration(position, 
				angle, etc..) change following values accordingly through testing*/
				const float NUMBER_SIGN_RECT_MIN_WIDTH = 15;
				const float NUMBER_SIGN_RECT_MAX_WIDTH = 25;
				const float NUMBER_SIGN_RECT_MIN_HEIGHT = 107;
				const float NUMBER_SIGN_RECT_MAX_HEIGHT = 120;

				const float ARROW_SIGN_RECT_MIN_WIDTH = 16;
				const float ARROW_SIGN_RECT_MAX_WIDTH = 20;
				const float ARROW_SIGN_RECT_MIN_HEIGHT = 125;
				const float ARROW_SIGN_RECT_MAX_HEIGHT = 137;

				const float SPEEDEND_SIGN_RECT_MIN_WIDTH = 40;
				const float SPEEDEND_SIGN_RECT_MAX_WIDTH = 58;
				const float SPEEDEND_SIGN_RECT_MIN_HEIGHT = 130;
				const float SPEEDEND_SIGN_RECT_MAX_HEIGHT = 145;

				const float MIN_DISTANCE_SPEED_LIMIT_NUM = 25;
				const float MAX_DISTANCE_SPEED_LIMIT_NUM = 35;

				const float PEDESTRIAN_ISLAND_MIN_HEIGHT = 88;
				const float PEDESTRIAN_ISLAND_MAX_HEIGHT = 98;
				const float PEDESTRIAN_ISLAND_MIN_WIDTH = 63;
				const float PEDESTRIAN_ISLAND_MAX_WIDTH = 73;

				const float CROPPING_HEIGHT_PEDESTRIAN_ISLAND = 250;

				const float SPEEDEND_CROPPING_WIDTH = 65;
				const float SPEEDEND_CROPPING_HEIGHT = 145;

				const float NUMBER_SIGN_CROPPING_WIDHT = 30;
				const float NUMBER_SIGN_CROPPING_HEIGHT = 120;

				const float TURN_SIGN_CROPPING_WIDTH = 30;
				const float TURN_SIGN_CROPPING_HEIGHT = 120;

				const float RESIZE_WIDTH = 20;
				const float RESIZE_HEIGHT = 20;

				const float BLUR_IMAGE_WIDTH = 1;
				const float BLUR_IMAGE_HEIGHT = 1;
				
				bool continueDetection = true;
				bool debugMode = true;
				bool isDetectionUnderProgress = false;
				std::string fullServiceName;
				std::string nameOfInputImageTopic;
				std::string locationOfDataFile;
				std::shared_ptr<ros::NodeHandle> nodeHandle;

				ros::Subscriber subscriber;
				ros::ServiceServer setDetectionFlagService;
				ros::Publisher debugImageResultPublisher;

				std::string rosImageToCVEncoding = "bgr8";

				cv::HOGDescriptor *hog;
				void constructHogObject();

				cv::Ptr<cv::ml::SVM> modelPtr;

				void cropAndCompressImage(const cv::RotatedRect &contourRect, const cv::Mat &originalImage, cv::Mat &resultImage,
										  const cv::Size &croppingSize);

				float findMinOrMaxInPoints(cv::Point2f points[], int sizeOfArray, AxisType axis, BoundaryType boundary);

				int predictSign(const cv::Mat &croppedImage);

				bool isContourANumber(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result);
				
				bool isContourATurnSign(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result);
				
				bool isContourASpeedEnd(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result);

				bool isContourAPedestrianIsland(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result);

				void alignImageToPedestrianIsland(const cv::Mat &originalImage, const cv::RotatedRect &rotRect, cv::Mat &result);

				void publishResultsForDebug(const cv::Mat &originalImage, const std::vector<SignsOnLaneResult> &resultsOnLane);

				void mergeOrphanNumbers(const std::vector<SignsOnLaneResult> &orphanNumbers, std::vector<SignsOnLaneResult> &results);

				float slopeOfLine(const cv::Point2f &point1, const cv::Point2f &point2);

			public:
				DetectSignsOnLane(ros::NodeHandle &nh, std::string locationOfDatFile, std::string nameOfRawImageTopic,
										std::string setDetectionFlagServiceName, bool isDebugMode = true);
				bool setDetectionFlagServiceCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
				void detectSignsFromRawImage(const sensor_msgs::Image &image);
				void init();
		};
	}
}

#endif