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

				const float SPEEDEND_CROP_RECT_WIDTH = 65;
				const float SPEEDEND_CROP_RECT_HEIGHT = 145;
				const float OTHERSIGN_CROP_RECT_WIDTH = 30;
				const float OTHERSIGN_CROP_RECT_HEIGHT = 120;

				const float RESIZE_WIDTH = 20;
				const float RESIZE_HEIGHT = 20;

				const float BLUR_IMAGE_WIDTH = 1;
				const float BLUR_IMAGE_HEIGHT = 1;
				
				bool continueDetection = false;
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

				// Variables for detectSignsFromRawImage function
				cv::Mat imageFromRaw;
				cv::Mat greyImage;
				cv::Mat thresholdImage;
				std::vector<std::vector<cv::Point>> contours;
    			std::vector<cv::Vec4i> hierarchy;
    			cv::RotatedRect rotatedRect;
    			bool isContourASign = false;
				bool isContourAZebraStripe = false;
				std::vector<std::shared_ptr<cv::RotatedRect>> orphanZebraStripes;
				std::vector<std::shared_ptr<ZebraCrossing>> zebraCrossings;
				std::vector<std::shared_ptr<BarredAreaStripe>> barredAreaStripes;
				std::vector<std::shared_ptr<BarredArea>> barredAreas;
				std::array<cv::Point2f,4> zebraCrossingBoundingBox;
				std::array<cv::Point,4> barredAreaBoundingBox;
				cv::Point2f centerOfZebraCrossing;
				cv::Point centerOfBarredArea;
    			cv::Mat boxPts;
    			cv::Size cropRect;
				cv::Mat croppedImage;
				sensor_msgs::Image debugImage;
				std::shared_ptr<cv_bridge::CvImage> tempCVImage;

    			// Variables for filterContorsForSigns function
    			cv::Size2f sizeOfRect;
    			float heightOfRect;
    			float widthOfRect;
    			float angleOfRect;
    			bool isArrow;
    			bool isNumber;
    			bool isSpeedEnd;
				bool filterContorsForSigns(cv::RotatedRect &contourRect, cv::Size &cropRectForSize);

				// Variables for preprocessBeforeSignDetection function
				cv::Point2f pointsOfRectCorners[4];
				float minX;
				float maxX;
				float minY;
				float maxY;
				float multi = 1.3;
				cv::Point2f center;
				cv::Size size;
				cv::Mat rotationMatrix2D;
				void preprocessBeforeSignDetection(cv::RotatedRect &contourRect, cv::Mat &image, cv::Mat &result, 
														cv::Size &cropRectForSize);

				float findMinOrMaxInPoints(cv::Point2f points[], int sizeOfArray, AxisType axis, BoundaryType boundary);

				// Variables for predictSign function
				std::vector<std::vector<cv::Point>> contoursToDraw;
				std::vector<float> descriptors;
				float predictionNumber;
				std::string labelString;
				void predictSign(cv::Mat &image, cv::Mat &croppedImage, std::vector<cv::Point> &contour, cv::RotatedRect &contourRect);

			public:
				DetectSignsOnLane(ros::NodeHandle &nh, std::string locationOfDatFile, std::string nameOfRawImageTopic,
										std::string setDetectionFlagServiceName);
				bool setDetectionFlagServiceCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
				void detectSignsFromRawImage(const sensor_msgs::Image &image);
				void init();
		};
	}
}

#endif