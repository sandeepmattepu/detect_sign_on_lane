#ifndef SIGNS_ON_LANE_RESULT
#define SIGNS_ON_LANE_RESULT

#include <opencv2/opencv.hpp>

namespace otto_car
{
	namespace lane_markings
	{
		//! Used to describes the type of the sign.
		enum SIGN_TYPE
		{
			SPEED_LIMIT_NUMBER = 0,		/*!< Speed limit number like 30, 60, ..*/
			END_OF_SPEED_LIMIT = 1,		/*!< Speed limit end sign */
			TURN_LEFT = 2,				/*!< Turn left sign */
			TURN_RIGHT = 3,				/*!< Turn right sign */
			ZEBRA_CROSSING = 4,			/*!< Zebra crossing sign */
			BARRED_AREA = 5,			/*!< Barred area sign */
			PEDESTRIAN_ISLAND = 6		/*!< Pedestrian island sign */
		};

		//! Details of signs are stored in this struct
		/*!
			The signs on the lane which are detected are finally represented by this struct.
		*/
		struct SignsOnLaneResult
		{
			//! Type of the sign.
			SIGN_TYPE signType;
			//! Value in the sign(For speed limit it will be speed limit value. For other signs this value will be -1).
			int value;
			//! Width of the sign.
			double width;
			//! Height of the sign.
			double height;
			//! Center of the sign.
			cv::Point2f center;
			//! Bounding box that surrounds the sign.
			/*!
				\image html BoundingBox.png
			*/
			cv::Point2f boundingBox[4];
		};	
	}
}

#endif