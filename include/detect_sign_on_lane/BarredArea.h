#ifndef BARRED_AREA_SIGN
#define BARRED_AREA_SIGN

#include <math.h>
#include <memory>
#include <opencv2/opencv.hpp>

namespace otto_car
{
	namespace lane_markings
	{
        /*! \class BarredAreaStripe BarredArea.h "BarredArea.h"
        *  \brief It represents a single parallelogram stripe in barred area.
        *  \image html barred_area_stripe.png
        * 
        *  This class represents a single parallelogram stripe in barred area.
        */
        class BarredAreaStripe
        {
            private:
                /**
                 * Minimum acceptable length of smaller side of parallelogram.
                */
                static constexpr float SMALLER_SIDE_MINIMUM_LENGTH = 36;
                /**
                 * Maximum acceptable length of larger side of parallelogram.
                */
                static constexpr float SMALLER_SIDE_MAXIMUM_LENGTH = 43;
                /**
                 * Minimum acceptable length of larger side of parallelogram.
                */
                static constexpr float LARGE_SIDE_MINIMUM_LENGTH = 66;
                /**
                 * Maximum acceptable length of larger side of parallelogram.
                */
                static constexpr float LARGE_SIDE_MAXIMUM_LENGTH = 74;
                /**
                 * Minimum acceptable acute angle of parallelogram.
                */
                static constexpr float MIN_ACUTE_ANGLE = 25;
                /**
                 * Maximum acceptable acute angle of parallelogram.
                */
                static constexpr float MAX_ACUTE_ANGLE = 34;
                /**
                 * Minimum acceptable obtuse angle of parallelogram.
                */
                static constexpr float MIN_OBTUSE_ANGLE = 145;
                /**
                 * Maximum acceptable obtuse angle of parallelogram.
                */
                static constexpr float MAX_OBTUSE_ANGLE = 152;
                static constexpr double PI  = 3.141592653589793238463;

                /**
                 * This angle is equal to angle made by the longest side of the parallelogram with image's XY axis(Note: Image X axis is positive towards right side
                 * and Y axis is positive towards downwards)
                 * \image html barred_area_stripe_angle.png
                */
                float angleOfBarredArea = 0;

                /**
                 * This holds vertices of the parallelogram. Order of points are bottom most point, point which makes longer side with bottom most point,
                 * top most point and remaining point.
                 * \image html barred_area_stripe_box.png
                */
                std::array<cv::Point,4> boundingBox;

                /**
                 * Center of the parallelogram.
                */
                cv::Point centerPoint;

            public:
                //! Constructs a BarredAreaStripe instance.
                /*!
                    This instance is usually created and provided by the isBarredAreaStripe() function.
                    \param contour Contour which is closer to parallelogram. 
                    \sa isBarredAreaStripe()
                */
                BarredAreaStripe(const std::vector<cv::Point> &contour);

                //! Returns value present in BarredAreaStripe::angleOfBarredArea.
                float getAngleOfStripe();

                //! Gives vertices of the barred area stripe(parallelogram).
                /*!
                    Order of the vertices would be same as BarredAreaStripe::boundingBox.
                    \param result The vertices will be assigned to this parameter.
                */
                void getBarredAreaStripe(std::array<cv::Point,4> &result);

                //! Returns center of the parallelogram.
                /*!
                    \param result The center of parallelogram is assigned to it.
                */
                void getCenterPoint(cv::Point &result);

                //! Checkes whether a given contour is a barred area stripe.
                /*!
                    \param contour Pass a contour that you want to check whether it is a barred area stripe or not.
                    \param result Instance of BarredAreaStripe is assigned to this pointer.
                    \return Whether the contour is a barred area stripe or not.
                */
                static bool isBarredAreaStripe(const std::vector<cv::Point> &contour, std::shared_ptr<BarredAreaStripe> &result);

                //! Measures distance between two points.
                static float distanceBtwPoints(const cv::Point2f &point1, const cv::Point2f &point2);

                //! Measures angle between three points.
                /*!
                    Measures angle between two lines formed by joining point1-point2 and point2-point3.
                */
                static float angleBtwThreePoints(const cv::Point &point1, const cv::Point &point2, const cv::Point &point3);
        };

        /*! \class BarredArea BarredArea.h "BarredArea.h"
         *  \brief It represents a barred area on the lane.
         *  \image html barred_area.png
         * 
         *  This class represents a barred area on the lane. It also has collection of BarredAreaStripe
         */
        class BarredArea
        {
            private:
                /**
                 * Minimum acceptable distance between BarredAreaStripe.
                */
                static constexpr float MIN_DISTANCE_BETWEEN_STRIPES = 59;

                /**
                 * Maximum acceptable distance between BarredAreaStripe.
                */
                static constexpr float MAX_DISTANCE_BETWEEN_STRIPES = 64;

                /**
                 * Minimum number of BarredAreaStripe that need to exist to call collection of BarredAreaStripe as BarredArea.
                */
                static constexpr int MIN_STRIPES_FOR_BARRED_AREA = 2;

                /**
                 * This is equal to any one of the stripe's BarredAreaStripe::angelOfBarredArea value
                */
                double angleOfBarredArea = 0;

                /**
                 * Width of the barred area(smaller side).
                 * \image html barred_area_width.png
                */
                double widthOfBarredArea = 0;

                /**
                 * Height of barred area(larger side).
                 * \image html barred_area_height.png
                */
                double heightOfBarredArea = 0;

                //! Stores collection of BarredAreaStripe that makes a barred area.
                /*! 
                    The order of barred area stripes that are stored are from bottom to top.
                    \image html barred_area_stripes_vector.png
                */
                std::vector<std::shared_ptr<BarredAreaStripe>> stripes;

                /**
                 * Center of barred area.
                */
                cv::Point center;

                //! Gives unit vector whose direction points from starting point towards ending point
                /*!
                    \param startingPoint Point from which vector is started(Tail).
                    \param endingPoint Point to which vector is ended(Head).
                    \param unitVectorResult Unit vector result is assigned to this parameter.
                */
                void unitVector(const cv::Point2f &startingPoint, const cv::Point2f &endingPoint, cv::Point2f &unitVectorResult);

                //! Calculates dimensions of barred area.
                /*!
                    Once a BarredAreaStripe is merged into this instance, new dimensions of barred area are calculated with this function.
                    \sa mergeAnotherBarredAreaStripe()
                */
                void calculateDimensionsOfBarredArea();

            public:
                //! Constructs an instance of BarredArea
                /*!
                    Usually instances of BarredArea are created and provide by clusterStripes() function.
                    \sa clusterStripes()
                */
                BarredArea();

                //! Clusters a set of BarredAreaStripe into BarredArea.
                /*!
                    After getting a set of BarredAreaStripe from contours through BarredAreaStripe::isBarredAreaStripe(). Use this function to
                    merge BarredAreaStripe which are closer together.
                    \param barredStripes Set of BarredAreaStripe.
                    \param results Set of BarredArea are assigned to this parameter.
                    \image html barred_area_cluster_stripes_1.png "Before merging the stripes"<br>
                    \image html barred_area_cluster_stripes_2.png "After merging the stripes"
                    \sa BarredAreaStripe::isBarredAreaStripe()
                */
                static void clusterStripes(std::vector<std::shared_ptr<BarredAreaStripe>> &barredStripes, std::vector<std::shared_ptr<BarredArea>> &results);
                
                //! Merge another BarredArea into this instance
                /*!
                    \param otherBarredArea Other BarredArea that need to be merged with this instance.
                    \image html barred_area_merge_1.png "Before merging the BarredArea"<br>
                    \image html barred_area_merge_2.png "After merging the BarredArea"
                    \sa clusterStripes()
                */
                bool mergeAnotherBarredAreaStripe(std::shared_ptr<BarredArea> &otherBarredArea);

                //! Calculated center of the barred area
                /*!
                    \sa calculateDimensionsOfBarredArea()
                */
                void calculateCenter();

                //! Gives bounding box of the barred area.
                /*!
                    This bounding box has little bit padding from the extremes of BarredAreaStripe to accomidate smaller traingles
                    at the end and make it look like it is surrounding entire barred area.
                    \param box Assigns vertices of bounding box of barred area to this parameter.
                    \image html barred_area_box.png "Difference between actual barred area box and barred area box"
                */
                void getBarredAreaBox(std::array<cv::Point2f,4> &box);

                //! Gives actual bounding box of barred area.
                /*!
                    This bounding box just surrounds the set of barred areas.
                    \image html barred_area_actual_box.png
                    \sa getBarredAreaBox()
                */
                void getActualBarredAreaBox(std::array<cv::Point2f,4> &box);

                //! Returns center of the barred area.
                /*!
                    \param centerResult Assigns center of barred area to this parameter.
                */
                void getCenter(cv::Point2f &centerResult);

                //! Returns BarredArea::widthOfBarredArea value.
                double getWidthOfBarredArea();

                //! Returns BarredArea::heightOfBarredArea value.
                double getHeightOfBarredArea();
        };
    }
}

#endif