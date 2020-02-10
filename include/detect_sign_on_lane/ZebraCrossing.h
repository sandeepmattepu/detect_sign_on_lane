#ifndef ZEBRA_CROSSING_SIGN
#define ZEBRA_CROSSING_SIGN

#include <math.h>
#include <memory>
#include "opencv2/opencv.hpp"

namespace otto_car
{
	namespace lane_markings
	{
        //! This class represents a zebra crossing sign
        /*!
            \image html ZebraCrossing.png
        */
        class ZebraCrossing
        {
            private:
                //! Minimum height of zebra crossing.
                static constexpr float ZEBRA_CROSSING_MINIMUM_HEIGHT = 90;
                //! Maximum height of zebra crossing.
                static constexpr float ZEBRA_CROSSING_MAXIMUM_HEIGHT = 118;
                //! Minimum width of zebra crossing.
                static constexpr float ZEBRA_CROSSING_MINIMUM_WIDTH = 7;
                //! Maximum width of zebra crossing.
                static constexpr float ZEBRA_CROSSING_MAXIMUM_WIDTH = 18;
                //! Minimum acceptable distance between zebra stripes.
                static constexpr float MIN_DISTANCE_BTW_ZEBRA_STRIPES = 14;
                //! Maximum acceptable distance between zebra stripes.
                static constexpr float MAX_DISTANCE_BTW_ZEBRA_STRIPES = 26;
                //! Minimum number of stripes a zebra crossing need to have to consider it to be valid zebra crossing.
                static constexpr int MIN_STRIPES_FOR_ZEBRA_CROSSING = 4;

                //! Centers of the stripes in zebra crossing.
                /*!
                    The stripe's centers of the zebra crossing are stored in this variable. Order of these stripe centers are from left to right.
                    \image html ZebraCrossing_Stripes.png
                */
                std::vector<std::shared_ptr<cv::Point2f>> stripeCenters;
                //! Angle of zebra crossing.
                /*!
                    This angle is equal to angle of any of it's stripe(Angle of stripe is derived from RotatedRect of the stripe).
                */
                float angleOfZebraCrossing = 0;
                //! Width of the zebra crossing
                /*!
                    \image html ZebraCrossing_Width.png
                */
                float widthOfZebraCrossing = 0;
                //! Height of zebra crossing
                /*!
                    \image html ZebraCrossing_Height.png
                */
                float heightOfZebraCrossing = 0;
                //! Center of entire zebra crossing.
                cv::Point2f centerOfZebraCrossing;
                //! Bounding box that surrounds entire zebra crossing
                /*! Order of vertices are BottomLeft, TopLeft, TopRight, BottomRight.
                    \image html ZebraCrossing_box.png
                */
                std::array<cv::Point2f,4> boundingBox;
                //! Calculated dimensions of zebra crossing.
                /*!
                    Once a zebra crossing is merged into another zebra crossing, this function is called to recalculate the dimension of the
                    new zebra crossing.
                    \sa mergeAnotherZebraCrossing()
                */
                void calculateDimensionsOfZebraCrossing();

            public:
                //! Construct an instance of zebra crossing.
                /*!
                    Usually instances of zebra crossing are created and provided by clusterStripesForZebraCrossings() function.
                    \sa clusterStripesForZebraCrossings()
                */
                ZebraCrossing();
                //! Checks whether given contour is a zebra stripe or not.
                /*!
                    \param contour Contour that needs to be checked.
                    \return Whether given contour is zebra stripe or not.
                */
                static bool isRectZebraStripe(const std::vector<cv::Point> &contour);
                //! Merges stripes into zebra crossings.
                /*!
                    The zebra crossing stripes are merged together to form a zebra crossing.
                    \image html ZebraCrossing_BeforeMerge.png "Before merging"<br>
                    \image html ZebraCrossing_AfterMerge.png "After merging"
                    \param stripes RotatedRects of conotours which got true for isRectZebraStripe() function.
                    \param results Results of zebra crossings are stored in this parameter.
                    \sa isRectZebraStripe()
                */
                static void clusterStripesForZebraCrossings(std::vector<std::shared_ptr<cv::RotatedRect>> &stripes,
                            std::vector<std::shared_ptr<ZebraCrossing>> &results);
                //! Returns a box that surrounds the zebra crossing.
                /*!
                    This function returns the values present in ZebraCrossing::boundingBox.
                    \param results The bounding box is assigned to this parameter.
                    \sa calculateDimensionsOfZebraCrossing()
                */
                void boundingBoxOfZebraCrossing(std::array<cv::Point2f,4> &results);
                //! Returns angle of zebra crossing.
                /*!
                    Returns ZebraCrossing::angleOfZebraCrossing value.
                    \return Angle of zebra crossing.
                    \sa calculateDimensionsOfZebraCrossing()
                */
                float getAngleOfZebraCrossing();
                //! Returns width of zebra crossing.
                /*!
                    Returns ZebraCrossing::widthOfZebraCrossing value.
                    \return Width of zebra crossing.
                    \sa calculateDimensionsOfZebraCrossing()
                */
                float getWidthOfZebraCrossing();
                //! Returns height of zebra crossing.
                /*!
                    Returns ZebraCrossing::heightOfZebraCrossing value.
                    \return Height of zebra crossing.
                    \sa calculateDimensionsOfZebraCrossing()
                */
                float getHeightOfZebraCrossing();
                //! Returns center of zebra crossing.
                /*!
                    Returns ZebraCrossing::centerOfZebraCrossing value.
                    \param result Center of zebra crossing is assigned to this parameter.
                    \sa calculateDimensionsOfZebraCrossing()
                */
                void getCenterOfZebraCrossing(cv::Point2f &result);
                //! Returns number of stripes in the zebra crossing.
                int getNumberOfStripes();
                //! Merges another zebra crossing into this zebra crossing.
                /*!
                    \image html ZebraCrossing_MergeOther_1.png "Before merge"<br>
                    \image html ZebraCrossing_MergeOther_2.png "After merge"
                    \param otherZebraCrossing Other zebra crossing that will be merged.
                    \return Whether the merge is successful or not.
                    \sa clusterStripesForZebraCrossings()
                */
                bool mergeAnotherZebraCrossing(std::shared_ptr<ZebraCrossing> otherZebraCrossing);
        };
    }
}

#endif