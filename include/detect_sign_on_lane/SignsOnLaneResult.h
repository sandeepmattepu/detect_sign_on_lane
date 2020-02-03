#ifndef SIGNS_ON_LANE_RESULT
#define SIGNS_ON_LANE_RESULT

#include <opencv2/opencv.hpp>

namespace otto_car
{
	namespace lane_markings
	{
		enum SIGN_TYPE
		{
			SPEED_LIMIT_NUMBER = 0,
			END_OF_SPEED_LIMIT = 1,
			TURN_LEFT = 2,
			TURN_RIGHT = 3,
			ZEBRA_CROSSING = 4,
			BARRED_AREA = 5,
			PEDESTRIAN_ISLAND = 6
		};

		struct SignsOnLaneResult
		{
			SIGN_TYPE signType;
			int value;
			double width;
			double height;
			cv::Point2f center;
			cv::Point2f boundingBox[4];
		};	
	}
}

#endif