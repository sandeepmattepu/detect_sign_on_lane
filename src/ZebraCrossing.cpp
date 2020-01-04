#include "detect_sign_on_lane/ZebraCrossing.h"

namespace otto_car
{
	namespace lane_markings
	{
        ZebraCrossing::ZebraCrossing()
        {

        }

        bool ZebraCrossing::isRectZebraStripe(cv::RotatedRect &rotatedRect)
        {
            bool result = false;
            cv::Size2f sizeOfRect = rotatedRect.size;
            float heightOfRect = sizeOfRect.height;
            float widthOfRect = sizeOfRect.width;
            if(widthOfRect > heightOfRect)
            {
                float temp = widthOfRect;
                widthOfRect = heightOfRect;
                heightOfRect = temp;
            }
            result = (widthOfRect <= ZEBRA_CROSSING_MAXIMUM_WIDTH) && (widthOfRect >= ZEBRA_CROSSING_MINIMUM_WIDTH);
            result = result && (heightOfRect <= ZEBRA_CROSSING_MAXIMUM_HEIGHT) && 
                        (heightOfRect >= ZEBRA_CROSSING_MINIMUM_HEIGHT);
            return result;
        }
    }
}