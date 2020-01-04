#ifndef ZEBRA_CROSSING_SIGN
#define ZEBRA_CROSSING_SIGN

#include "opencv2/opencv.hpp"

namespace otto_car
{
	namespace lane_markings
	{
        class ZebraCrossing
        {
            private:

                static constexpr float ZEBRA_CROSSING_MINIMUM_HEIGHT = 95;
                static constexpr float ZEBRA_CROSSING_MAXIMUM_HEIGHT = 111;
                static constexpr float ZEBRA_CROSSING_MINIMUM_WIDTH = 9;
                static constexpr float ZEBRA_CROSSING_MAXIMUM_WIDTH = 15;

            public:
                
                ZebraCrossing();
                static bool isRectZebraStripe(cv::RotatedRect &rotatedRect);
        };
    }
}

#endif