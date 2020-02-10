#ifndef DETECT_SIGNS_ON_LANE
#define DETECT_SIGNS_ON_LANE

#include <string>
#include <array>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "detect_sign_on_lane/SignsOnLaneMsg.h"

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
		//! Type of axis
		/*!
			This enum is used to describe the axis type. It is usually used to find max or min of points.
			\sa DetectSignsOnLane::findMinOrMaxInPoints()
		*/
		enum AxisType 
		{
			X,	/*!< X-axis */
			Y	/*!< Y-axis */
		};

		//! Type of boundary
		/*!
			This enum is used to describe comparision type i.e maximum or minimum. It is usually used to find max
			or min of points.
			\sa DetectSignsOnLane::findMinOrMaxInPoints()
		*/
		enum BoundaryType 
		{
			MIN,	/*!< Minimum comparator */
			MAX		/*!< Maximum comparator */
		};

		//! Detects the signs on the lane
		/*!
			This class subscribes to the topic that publishes the top view image of the lane, it detects the signs on the
			lane and publish the results on other topic name.
		*/
		class DetectSignsOnLane
		{
			private:
				/* The following constrain values are obtained by testing. If you alter the camera configuration(position, 
				angle, etc..) change following values accordingly through testing*/

				//! Minimum allowed width of the number sign.
				const float NUMBER_SIGN_RECT_MIN_WIDTH = 15;
				//! Maximum allowed width of the number sign.
				const float NUMBER_SIGN_RECT_MAX_WIDTH = 25;
				//! Minimum allowed height of the number sign.
				const float NUMBER_SIGN_RECT_MIN_HEIGHT = 107;
				//! Maximum allowed height of the number sign.
				const float NUMBER_SIGN_RECT_MAX_HEIGHT = 120;

				//! Minimum allowed width of the arrow sign.
				const float ARROW_SIGN_RECT_MIN_WIDTH = 16;
				//! Maximum allowed width of the arrow sign.
				const float ARROW_SIGN_RECT_MAX_WIDTH = 20;
				//! Minimum allowed height of the arrow sign.
				const float ARROW_SIGN_RECT_MIN_HEIGHT = 125;
				//! Maximum allowed height of the arrow sign.
				const float ARROW_SIGN_RECT_MAX_HEIGHT = 137;

				//! Minimum allowed width of speed end sign.
				const float SPEEDEND_SIGN_RECT_MIN_WIDTH = 40;
				//! Maximum allowed width of speed end sign.
				const float SPEEDEND_SIGN_RECT_MAX_WIDTH = 58;
				//! Minimum allowed height of speed end sign.
				const float SPEEDEND_SIGN_RECT_MIN_HEIGHT = 130;
				//! Maximum allowed height of speen dend sign.
				const float SPEEDEND_SIGN_RECT_MAX_HEIGHT = 145;

				//! Minimum allowed distance between two numbers in speed limit sign.
				const float MIN_DISTANCE_SPEED_LIMIT_NUM = 15;
				//! Maximum allowed distance between two numbers in speed limit sign.
				const float MAX_DISTANCE_SPEED_LIMIT_NUM = 35;

				//! Minimum allowed height of pedestrian island.
				const float PEDESTRIAN_ISLAND_MIN_HEIGHT = 88;
				//! Maximum allowed height of pedestrian island
				const float PEDESTRIAN_ISLAND_MAX_HEIGHT = 98;
				//! Minimum allowed width of pedestrian island.
				const float PEDESTRIAN_ISLAND_MIN_WIDTH = 63;
				//! Maximum allowed width of pedestrian island.
				const float PEDESTRIAN_ISLAND_MAX_WIDTH = 73;

				/**
				 * After a contour matches dimensions of the pedestrian island then image need to be cropped
				 * upto pedestrian island. This constant determines the height of cropping.
				*/
				const float CROPPING_HEIGHT_PEDESTRIAN_ISLAND = 250;

				//! Width of the cropping box to crop a speed end sign.
				const float SPEEDEND_CROPPING_WIDTH = 55;
				//! Height of the cropping box to crop a speed end sign.
				const float SPEEDEND_CROPPING_HEIGHT = 145;

				//! Width of the cropping box to crop a number sign.
				const float NUMBER_SIGN_CROPPING_WIDHT = 30;
				//! Height of the cropping box to crop a number sign.
				const float NUMBER_SIGN_CROPPING_HEIGHT = 120;

				//! Width of the cropping box to crop a turn sign.
				const float TURN_SIGN_CROPPING_WIDTH = 30;
				//! Height of the cropping box to crop a turn sign.
				const float TURN_SIGN_CROPPING_HEIGHT = 120;

				//! The amount of cropped image's width to resize.
				const float RESIZE_WIDTH = 20;
				//! The amount of cropped image's height to resize.
				const float RESIZE_HEIGHT = 20;

				const float BLUR_IMAGE_WIDTH = 1;
				const float BLUR_IMAGE_HEIGHT = 1;

				//! Flag to determine whether to perform sign detection or not.				
				bool continueDetection = true;
				//! Flag to determine whether to publish optional debug message which has detected messages drawn on them.
				bool debugMode = true;
				//! Determines if DetectSignsOnLane::detectSignsFromRawImage() is under progress. This avoids multiple calls to DetectSignsOnLane::detectSignsFromRawImage().
				bool isDetectionUnderProgress = false;
				//! Name of the service which enables/disables detection of signs on the lane.
				std::string fullServiceName;
				//! Name of the topic which is top view of the lane.
				std::string nameOfInputImageTopic;
				//! Location of the trained svm model file.
				std::string locationOfDataFile;
				
				//! Node handle to which this instance advertise/subscribes to topics.
				std::shared_ptr<ros::NodeHandle> nodeHandle;
				//! Ros subscriber which subscribes to top view of the lane.
				ros::Subscriber subscriber;
				//! Ros service which offers functionality to enable/disable detection.
				ros::ServiceServer setDetectionFlagService;
				//! Ros publisher on which this instance publishes the detected signs on the lane.
				ros::Publisher signResultsPublisher;
				//! Ros publisher on which this instance publishes the top view of lane with results drawn on them.
				ros::Publisher debugImageResultPublisher;

				std::string rosImageToCVEncoding = "bgr8";
				cv::HOGDescriptor *hog;
				//! Model which classifies the sign.
				cv::Ptr<cv::ml::SVM> modelPtr;

				//! Constructs HogDescriptor and assigns to DetectSignsOnLane::hog variable.
				void constructHogObject();

				//! Crops and compresses an image.
				/*!
					If a contour matches any sign dimensions then they need to be cropped and compressed to predict the sign.
					This function crops and then resizes the image based on DetectSignsOnLane::RESIZE_WIDTH and 
					DetectSignsOnLane::RESIZE_HEIGHT
					\param contourRect RotatedRect of the contour.
					\param originalImage Image from which newly cropped and compressed image is formed.
					\param resultImage Result image is assigned to this parameter.
					\param croppingSize Cropping dimensions.
					\sa isContourANumber(), isContourATurnSign(), isContourASpeedEnd()
				*/
				void cropAndCompressImage(const cv::RotatedRect &contourRect, const cv::Mat &originalImage, cv::Mat &resultImage,
										  const cv::Size &croppingSize);

				//! Finds a minimum or maximum among given points.
				/*!
					\param points Array of points to compare.
					\param sizeOfArray size of the points array.
					\param axis Which axis to compare.
					\param boundary Maximum or minimum comparator to use.
					\return X/Y(based on parameter) value of point which is maximum/minimum(based on parameter).
				*/
				float findMinOrMaxInPoints(cv::Point2f points[], int sizeOfArray, AxisType axis, BoundaryType boundary);

				//! Predicts the sign.
				/*!
					After an image is cropped and resized, the sign in the image need to be predicted. This function
					predicts the sign in the image.
					\param croppedImage The cropped and resized image which has a sign
					\return Number which means a particular sign.<br>
					[0] : Number sign "0".<br>
					[1] : Number sign "1".<br>
					[2] : Number sign "2".<br>
					[3] : Number sign "3".<br>
					[4] : Number sign "4".<br>
					[5] : Number sign "5".<br>
					[6] : Number sign "6".<br>
					[7] : Number sign "7".<br>
					[8] : Number sign "8".<br>
					[9] : Number sign "9".<br>
					[10] : Speed limit ends.<br>
					[11] : Turn left.<br>
					[12] : Turn right.<br>
					[13] : False detections.<br>
					[14] : Inverted number.<br>
					[15] : Pedestrian island.
					\sa isContourANumber(), isContourATurnSign(), isContourASpeedEnd(), isContourAPedestrianIsland()
				*/
				int predictSign(const cv::Mat &croppedImage);

				//! Checks whether a contour is a number sign or not.
				/*!
					\param originalImage Top view of the image which might have a sign.
					\param contour Contour which need to be checked whether it is a number sign or not.
					\param result If there is a number sign on the lane, then it's details are assigned to this parameter.
					\return Whether a contour is a number sign or not.
				*/
				bool isContourANumber(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result);
				
				//! Checks whether a contour is a turn sign or not.
				/*!
					\param originalImage Top view of the image which might have a sign.
					\param contour Contour which need to be checked whether it is a turn sign or not.
					\param result If there is a turn sign on the lane, then it's details are assigned to this parameter.
					\return Whether a contour is a turn sign or not.
				*/
				bool isContourATurnSign(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result);
				
				//! Checks whether a contour is a speed end or not.
				/*!
					\param originalImage Top view of image which might have a sign.
					\param contour Contour which need to be checked whether it is a speed end or not.
					\param result If there is a speed end on the lane, then it's details are assigned to this parameter.
					\return Whether a contour is speed end or not.
				*/
				bool isContourASpeedEnd(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result);

				//! Checks whether a contour is a pedestrian island or not.
				/*!
					\param originalImage Top view of image which might have a sign.
					\param contour Contour which need to be checked whether it is a pedestrian island.
					\param result If there is a pedestrian island, then it's details are assigned to this parameter.
					\return Whether a contour is a pedestrian island or not.
				*/
				bool isContourAPedestrianIsland(const cv::Mat &originalImage, const std::vector<cv::Point> &contour, SignsOnLaneResult &result);

				//! Align an image to pedestrian island.
				/*!
					Entire image is rotated to align pedestrian island in upright position.
					\image html AlignPedestrianIsland.png "Before aligning the image"<br>
					\image html AlignPedestrianIsland_2.png "After aligning the image"
					\param originalImage Image that needed to be rotated.
					\param rotRect RotatedRect of contour which is a pedestrian island.
					\param result Rotated image is assigned to this result parameter.
					\sa isContourAPedestrianIsland()
				*/
				void alignImageToPedestrianIsland(const cv::Mat &originalImage, const cv::RotatedRect &rotRect, cv::Mat &result);

				//! Results of signs that are detected are published under a topic name.
				/*!
					The signs that are detected in an image are published under a topic name for other nodes to use.
					\param resultsOnLane Set of signs on the lane.
					\sa detectSignsFromRawImage()
				*/
				void publishResults(const std::vector<SignsOnLaneResult> &resultsOnLane);

				//! Results of signs are drawn on the topview image and published.
				/*!
					The signs that are detected in an image are drawn on the top view image and this image is published as
					a message. Publishing of the debug image is dependent on DetectSignsOnLane::debugMode flag.
					\param originalImage Image on which sign results are drawn.
					\param resultsOnLane Sign results.
					\sa detectSignsFromRawImage()
				*/
				void publishResultsForDebug(const cv::Mat &originalImage, const std::vector<SignsOnLaneResult> &resultsOnLane);

				//! Merge individual number sign into signs which has two numbers.
				/*!
					The individual number sign have to be merged since speed limit numbers in real life are two digit numbers.
					\image html OrphanImage_1.png "Before merging individual numbers"<br>
					\image html OrphanImage_2.png "After merging the numbers"
					\param orphanNumbers The numbers which needed to be merged.
					\param results The resulting two digit numbers are assigned to this value.
					\sa detectSignsFromRawImage(), isContourANumber()
				*/
				void mergeOrphanNumbers(const std::vector<SignsOnLaneResult> &orphanNumbers, std::vector<SignsOnLaneResult> &results);

				//! Measures slope of line formed by two points.
				/*!
					This slope is measured with respect to image X-Y axis(X axis is positive towards right and Y axis is positive
					toward left).
				*/
				float slopeOfLine(const cv::Point2f &point1, const cv::Point2f &point2);

			public:
				//! Constructs DetectSignsOnLane instance.
				/*!
					\param nh Ros NodeHandle is required for this instance to subscribe/publish to the topics.
					\param locationOfDatFile Path location where svm model file is present in the system.
					\param nameOfRawImageTopic Name of the topic which has top view of the lane.
					\param setDetectionFlagServiceName Name of the service which enables/disables detection of signs on lane.
					\param isDebugMode Determines whether to publish debug image with detected signs on them or not.
				*/
				DetectSignsOnLane(ros::NodeHandle &nh, std::string locationOfDatFile, std::string nameOfRawImageTopic,
										std::string setDetectionFlagServiceName, bool isDebugMode = true);

				//! Function which enables or disables the detection of signs on the lane.
				/*!
					This function gets called whenever other nodes that wants to enable/disable detection of signs on the lane
					under service name DetectSignsOnLane::fullServiceName.
				*/
				bool setDetectionFlagServiceCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

				//! Function which recieves top view of the lane.
				/*!
					This function gets called since this instance is subscribed to the DetectSignOnLane::nameOfInputImageTopic topic.
				*/
				void detectSignsFromRawImage(const sensor_msgs::Image &image);

				//! Initializes important variables in this instance.
				/*!
					After constructing a new instance of DetectSignsOnLane object, it is important to initialize it using this function.
					\sa DetectSignsOnLane()
				*/
				void init();
		};
	}
}

#endif