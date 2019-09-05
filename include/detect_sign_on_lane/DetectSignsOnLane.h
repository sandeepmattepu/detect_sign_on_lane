#ifndef DETECT_SIGNS_ON_LANE
#define DETECT_SIGNS_ON_LANE

#include <string>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect.hpp"

/* The following constrain values are obtained by testing. If you alter the camera configuration(position, angle, etc..)
   change following values accordingly through testing*/
#define NUMBER_SIGN_RECT_MIN_WIDTH 15
#define NUMBER_SIGN_RECT_MAX_WIDTH 25
#define NUMBER_SIGN_RECT_MIN_HEIGHT 110
#define NUMBER_SIGN_RECT_MAX_HEIGHT 120

#define ARROW_SIGN_RECT_MIN_WIDTH 16
#define ARROW_SIGN_RECT_MAX_WIDTH 20
#define ARROW_SIGN_RECT_MIN_HEIGHT 125
#define ARROW_SIGN_RECT_MAX_HEIGHT 137

#define SPEEDEND_SIGN_RECT_MIN_WIDTH 40
#define SPEEDEND_SIGN_RECT_MAX_WIDTH 58
#define SPEEDEND_SIGN_RECT_MIN_HEIGHT 130
#define SPEEDEND_SIGN_RECT_MAX_HEIGHT 145

namespace otto_car
{
	namespace lane_markings
	{
		/**
		* This class is responsible for detecting the code 
		*/
		class DetectSignsOnLane
		{
			private:
				bool continueDetection = false;

				std::string setDetectionFlagServiceName = "/set_sign_detection_on_lane_flag";
				ros::ServiceServer setDetectionFlagService;

				std::string rosImageToCVEncoding = "bgr8";

				cv::HOGDescriptor hog;
				cv::HOGDescriptor constructHogObject();

				// Variables for detectSignsFromRawImage function
				cv::Mat imageFromRaw;
				cv::Mat greyImage;
				cv::Mat thresholdImage;
				std::vector<std::vector<cv::Point> > contours;
    			std::vector<cv::Vec4i> hierarchy;
    			bool isContourSign = false;

    			// Variables for filterContorsForSigns function
    			cv::RotatedRect rotatedRect;
    			cv::Size2f sizeOfRect;
    			float heightOfRect;
    			float widthOfRect;
    			float angleOfRect;
    			bool isArrow;
    			bool isNumber;
    			bool isSpeedEnd;

			public:
				DetectSignsOnLane(ros::NodeHandle &nh, std::string nameOfLaneImageTopic);
				bool setDetectionFlagServiceCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
				void detectSignsFromRawImage(const sensor_msgs::Image &image);
				bool filterContorsForSigns(std::vector<cv::Point> contour);
		};
	}
}

#endif