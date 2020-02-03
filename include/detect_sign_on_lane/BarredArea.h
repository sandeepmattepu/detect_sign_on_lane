#ifndef BARRED_AREA_SIGN
#define BARRED_AREA_SIGN

#include <math.h>
#include <memory>
#include <opencv2/opencv.hpp>

namespace otto_car
{
	namespace lane_markings
	{


        class BarredAreaStripe
        {
            private:

                static constexpr float SMALLER_SIDE_MINIMUM_LENGTH = 36;
                static constexpr float SMALLER_SIDE_MAXIMUM_LENGTH = 43;
                static constexpr float LARGE_SIDE_MINIMUM_LENGTH = 66;
                static constexpr float LARGE_SIDE_MAXIMUM_LENGTH = 74;
                static constexpr float MIN_ACUTE_ANGLE = 25;
                static constexpr float MAX_ACUTE_ANGLE = 34;
                static constexpr float MIN_OBTUSE_ANGLE = 145;
                static constexpr float MAX_OBTUSE_ANGLE = 152;
                static constexpr double PI  = 3.141592653589793238463;

                float angleOfBarredArea = 0;        // Angle is equal to the angle of the longest side

                // Order of bounding box is bottom most point, point which makes longer side with bottom most point,
                // top most point and remaining point
                std::array<cv::Point,4> boundingBox;
                cv::Point centerPoint;
            public:
                BarredAreaStripe(const std::vector<cv::Point> &contour);
                float getAngleOfStripe();
                void getBarredAreaStripe(std::array<cv::Point,4> &result);
                void getCenterPoint(cv::Point &result);
                static bool isBarredAreaStripe(const std::vector<cv::Point> &contour, std::shared_ptr<BarredAreaStripe> &result);
                static float distanceBtwPoints(const cv::Point &point1, const cv::Point &point2);
                static float angleBtwThreePoints(const cv::Point &point1, const cv::Point &point2, const cv::Point &point3);
        };

        class BarredArea
        {
            private:
                static constexpr float MIN_DISTANCE_BETWEEN_STRIPES = 59;
                static constexpr float MAX_DISTANCE_BETWEEN_STRIPES = 64;
                static constexpr int MIN_STRIPES_FOR_BARRED_AREA = 2;

                double angleOfBarredArea = 0;
                double widthOfBarredArea = 0;
                double heightOfBarredArea = 0;
                // Order of stripes are from bottom to top i.e bottom most stripe is at stripes[0]
                std::vector<std::shared_ptr<BarredAreaStripe>> stripes;
                cv::Point center;
                // Gives unit vector whose direction points from starting point towards ending point
                void unitVector(const cv::Point2f &startingPoint, const cv::Point2f &endingPoint, cv::Point2f &unitVectorResult);
                void calculateDimensionsOfBarredArea();

            public:
                BarredArea();
                static void clusterStripes(std::vector<std::shared_ptr<BarredAreaStripe>> &barredStripes, std::vector<std::shared_ptr<BarredArea>> &results);
                bool mergeAnotherBarredAreaStripe(std::shared_ptr<BarredArea> &otherBarredArea);
                void calculateCenter();
                void getBarredAreaBox(std::array<cv::Point2f,4> &box);
                void getActualBarredAreaBox(std::array<cv::Point2f,4> &box);
                void getCenter(cv::Point2f &centerResult);
                double getWidthOfBarredArea();
                double getHeightOfBarredArea();
        };
    }
}

#endif