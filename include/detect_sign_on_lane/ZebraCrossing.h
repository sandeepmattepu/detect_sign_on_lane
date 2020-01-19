#ifndef ZEBRA_CROSSING_SIGN
#define ZEBRA_CROSSING_SIGN

#include <math.h>
#include <memory>
#include "opencv2/opencv.hpp"

namespace otto_car
{
	namespace lane_markings
	{
        class ZebraCrossing
        {
            private:

                static constexpr float ZEBRA_CROSSING_MINIMUM_HEIGHT = 90;
                static constexpr float ZEBRA_CROSSING_MAXIMUM_HEIGHT = 118;
                static constexpr float ZEBRA_CROSSING_MINIMUM_WIDTH = 7;
                static constexpr float ZEBRA_CROSSING_MAXIMUM_WIDTH = 18;
                static constexpr float MIN_DISTANCE_BTW_ZEBRA_STRIPES = 14;
                static constexpr float MAX_DISTANCE_BTW_ZEBRA_STRIPES = 26;
                static constexpr int MIN_STRIPES_FOR_ZEBRA_CROSSING = 4;

                // Stored from left to right i.e left most stripe will be in stripeCenters[0]
                std::vector<std::shared_ptr<cv::Point2f>> stripeCenters;
                float angleOfZebraCrossing = 0;
                float widthOfZebraCrossing = 0;
                float heightOfZebraCrossing = 0;
                cv::Point2f centerOfZebraCrossing;
                // Order of the bounding box is BottomLeft, TopLeft, TopRight, BottomRight with respect to zebra crossing
                std::array<cv::Point2f,4> boundingBox;

                void calculateDimensionsOfZebraCrossing();

            public:
                
                ZebraCrossing();
                static bool isRectZebraStripe(cv::RotatedRect &rotatedRect);
                static void clusterStripesForZebraCrossings(std::vector<std::shared_ptr<cv::RotatedRect>> &stripes,
                            std::vector<std::shared_ptr<ZebraCrossing>> &results);
                void boundingBoxOfZebraCrossing(std::array<cv::Point2f,4> &results);
                float getAngleOfZebraCrossing();
                float getWidthOfZebraCrossing();
                float getHeightOfZebraCrossing();
                void getCenterOfZebraCrossing(cv::Point2f &result);
                int getNumberOfStripes();
                bool mergeAnotherZebraCrossing(std::shared_ptr<ZebraCrossing> otherZebraCrossing);
        };
    }
}

#endif