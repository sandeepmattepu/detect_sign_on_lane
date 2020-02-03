#ifndef SIGNS_ON_LANE_RESULT
#define SIGNS_ON_LANE_RESULT

#include <opencv2/opencv.hpp>

namespace otto_car
{
	namespace lane_markings
	{
		enum SIGN_TYPE
		{
			SPEED_LIMIT_NUMBER,
			END_OF_SPEED_LIMIT,
			TURN_LEFT,
			TURN_RIGHT,
			ZEBRA_CROSSING,
			BARRED_AREA,
			PEDESTRIAN_ISLAND
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