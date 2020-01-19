#include "detect_sign_on_lane/BarredArea.h"

namespace otto_car
{
    namespace lane_markings
	{
        BarredAreaStripe::BarredAreaStripe(const std::vector<cv::Point> &contour)
        {
            int indexOfBottomMostPoint = 0;
            float yPosOfBottomMostPoint = 0;
            for(int i = 0; i < boundingBox.size(); i++)
            {
                if(i == 0)
                {
                    yPosOfBottomMostPoint = contour[i].y;
                }
                else if(contour[i].y > yPosOfBottomMostPoint)        // Image's y axis is reverse direction
                {
                    yPosOfBottomMostPoint = contour[i].y;
                    indexOfBottomMostPoint = i;
                }
            }
            boundingBox[0].x = contour[indexOfBottomMostPoint].x;
            boundingBox[0].y = contour[indexOfBottomMostPoint].y;

            // Determining the point beside bottom most point which forms a longer side.
            int indexOfPointBesideBottomPoint_1 = indexOfBottomMostPoint + 1;
            indexOfPointBesideBottomPoint_1 %= 4;
            int indexOfPointBesideBottomPoint_2 = indexOfBottomMostPoint - 1;
            indexOfPointBesideBottomPoint_2 %= 4;
            float distBtwBottomPointAndPoint1 = distanceBtwPoints(contour[indexOfBottomMostPoint], contour[indexOfPointBesideBottomPoint_1]);
            float distBtwBottomPointAndPoint2 = distanceBtwPoints(contour[indexOfBottomMostPoint], contour[indexOfPointBesideBottomPoint_2]);
            if(distBtwBottomPointAndPoint1 > distBtwBottomPointAndPoint2)
            {
                boundingBox[1].x = contour[indexOfPointBesideBottomPoint_1].x;
                boundingBox[1].y = contour[indexOfPointBesideBottomPoint_1].y;
                indexOfPointBesideBottomPoint_1 += 1;
                indexOfPointBesideBottomPoint_1 %= 4;
                boundingBox[2].x = contour[indexOfPointBesideBottomPoint_1].x;
                boundingBox[2].y = contour[indexOfPointBesideBottomPoint_1].y;
                indexOfPointBesideBottomPoint_1 += 1;
                indexOfPointBesideBottomPoint_1 %= 4;
                boundingBox[3].x = contour[indexOfPointBesideBottomPoint_1].x;
                boundingBox[3].y = contour[indexOfPointBesideBottomPoint_1].y;
            }
            else
            {
                boundingBox[1].x = contour[indexOfPointBesideBottomPoint_2].x;
                boundingBox[1].y = contour[indexOfPointBesideBottomPoint_2].y;
                indexOfPointBesideBottomPoint_2 += 3;
                indexOfPointBesideBottomPoint_2 %= 4;
                boundingBox[2].x = contour[indexOfPointBesideBottomPoint_2].x;
                boundingBox[2].y = contour[indexOfPointBesideBottomPoint_2].y;
                indexOfPointBesideBottomPoint_2 += 3;
                indexOfPointBesideBottomPoint_2 %= 4;
                boundingBox[3].x = contour[indexOfPointBesideBottomPoint_2].x;
                boundingBox[3].y = contour[indexOfPointBesideBottomPoint_2].y;
            }
            float xElem = boundingBox[0].x - boundingBox[1].x;
            float yElem = boundingBox[0].y - boundingBox[1].y;
            angleOfBarredArea = atan(yElem / xElem);
            angleOfBarredArea = (angleOfBarredArea * 180 / PI);
            if(angleOfBarredArea < 0)
            {
                angleOfBarredArea += 180;
            }

            centerPoint.x = (boundingBox[0].x + boundingBox[2].x) / 2;
            centerPoint.y = (boundingBox[0].y + boundingBox[2].y) / 2;
        }

        float BarredAreaStripe::getAngleOfStripe()
        {
            return angleOfBarredArea;
        }

        void BarredAreaStripe::getBarredAreaStripe(std::array<cv::Point,4> &result)
        {
            for(int i = 0; i < boundingBox.size(); i++)
            {
                result[i].x = boundingBox[i].x;
                result[i].y = boundingBox[i].y;
            }
        }

        void BarredAreaStripe::getCenterPoint(cv::Point &result)
        {
            result.x = centerPoint.x;
            result.y = centerPoint.y;
        }

        bool BarredAreaStripe::isBarredAreaStripe(const std::vector<cv::Point> &contour, std::shared_ptr<BarredAreaStripe> &barredAreaStripe)
        {
            bool result = false;

            // Check whether approximated contour has four sides
            float epsilonCoEff = 0.05;
			double arcLength = cv::arcLength(contour, true);
            std::vector<cv::Point> approxPolyPoints;
			cv::approxPolyDP(contour, approxPolyPoints, epsilonCoEff * arcLength, true);
            if(approxPolyPoints.size() == 4)
            {
                // Check whether sides are of appropriate length
                int numOfSmallerSides = 0;
                int numOfLargerSides = 0;
                for(int i = 0; i < approxPolyPoints.size(); i++)
                {
                    float lengthOfSide = distanceBtwPoints(approxPolyPoints[i], approxPolyPoints[(i+1)%4]);
                    if((lengthOfSide >= SMALLER_SIDE_MINIMUM_LENGTH) && (lengthOfSide <= SMALLER_SIDE_MAXIMUM_LENGTH))
                    {
                        numOfSmallerSides += 1;
                    }
                    else if((lengthOfSide >= LARGE_SIDE_MINIMUM_LENGTH) && (lengthOfSide <= LARGE_SIDE_MAXIMUM_LENGTH))
                    {
                        numOfLargerSides += 1;
                    }
                }

                if(numOfSmallerSides == 2 && numOfLargerSides == 2)
                {
                    // Check whether they have proper angles between them
                    bool hasProperAcuteAngle = false;
                    bool hasProperObtuseAngle = false;
                    for(int i = 0; i < 2; i++)
                    {
                        float angleBtwPoints = angleBtwThreePoints(approxPolyPoints[i], approxPolyPoints[i+1], approxPolyPoints[i+2]);
                        if((angleBtwPoints >= MIN_ACUTE_ANGLE) && (angleBtwPoints <= MAX_ACUTE_ANGLE))
                        {
                            hasProperAcuteAngle = true;
                        }
                        else if((angleBtwPoints >= MIN_OBTUSE_ANGLE) && (angleBtwPoints <= MAX_OBTUSE_ANGLE))
                        {
                            hasProperObtuseAngle = true;
                        }
                    }

                    if(hasProperObtuseAngle && hasProperAcuteAngle)
                    {
                        result = true;
                        barredAreaStripe = std::make_shared<BarredAreaStripe>(approxPolyPoints);
                    }
                }
            }
            return result;
        }

        float BarredAreaStripe::distanceBtwPoints(const cv::Point &point1, const cv::Point &point2)
        {
            float xComponent, yComponent;
			xComponent = (point1.x - point2.x) * (point1.x - point2.x);
			yComponent = (point1.y - point2.y) * (point1.y - point2.y);
			return sqrt(xComponent + yComponent);
        }

        float BarredAreaStripe::angleBtwThreePoints(const cv::Point &point1, const cv::Point &point2, const cv::Point &point3)
        {
            float vec1XComponent = point1.x - point2.x;
            float vec1YComponent = point1.y - point2.y;
            float vec2XComponent = point2.x - point3.x;
            float vec2YComponent = point2.y - point3.y;

            float vec1Magnitude = sqrt((vec1XComponent  * vec1XComponent) + (vec1YComponent * vec1YComponent));
            float vec2Magnitude = sqrt((vec2XComponent * vec2XComponent) + (vec2YComponent * vec2YComponent));

            float dotProduct = (vec1XComponent * vec2XComponent) + (vec1YComponent * vec2YComponent);

            float angleInRadians = acos(dotProduct / (vec1Magnitude * vec2Magnitude));
            return (angleInRadians * 180 / PI);
        }

        void BarredArea::clusterStripes(std::vector<std::shared_ptr<BarredAreaStripe>> &barredStripes,
                                                    std::vector<std::shared_ptr<BarredArea>> &results)
        {
            if(barredStripes.size() == 0)
            {
                return;
            }

            std::vector<std::shared_ptr<BarredArea>> tempResults;
            for(int i = 0; i < barredStripes.size(); i++)         // Treat each stripe as a barred area
            {
                std::shared_ptr<BarredArea> tempBarredArea = std::make_shared<BarredArea>();
                if(barredStripes[i] != nullptr)
                {
                    tempBarredArea->stripes.push_back(barredStripes[i]);
                    tempBarredArea->angleOfBarredArea = barredStripes[i]->getAngleOfStripe();
                    tempBarredArea->calculateCenter();
                    tempResults.push_back(tempBarredArea);
                }
            }

            /* 1. Barred area stipes are combined together when distance between them is appropriate.
               2. As we know distance is measured between two elements(stripeA to stripeB). StripeA is selected
                  from outer for loop and stripeB is selected from the inner for loop.
               3. When distance between StripeA and StripeB is appropriate StripeB is removed and StripeA has
                  dimensions of combined old StripeA and removed StripeB.
               4. If there is atleast one merge with StripeA then StripeA will be unchanged and it is again
                  compared with new StripeB elements.
               5. If there are no merges then new StripeA element is selected and above steps 3-4 is done.
            */
            for(int i = 0; i < (tempResults.size() - 1); i++)
            {
                std::shared_ptr<BarredArea> stripeA = tempResults[i];
                int numberOfMerges = 1;
                while(numberOfMerges > 0)
                {
                    numberOfMerges = 0;
                    for(int j = (i + 1); j < tempResults.size(); j++)
                    {
                        std::shared_ptr<BarredArea> stripeB = tempResults[j];
                        if(stripeA->mergeAnotherBarredAreaStripe(stripeB))
                        {
                            tempResults.erase(tempResults.begin() + j);
                            numberOfMerges += 1;
                        }
                    }
                }
            }

            for(int i = 0; i < tempResults.size(); i++)
            {
                // Zebra crossing which has valid number of stripes are added to the result
                if(tempResults[i]->stripes.size() >= MIN_STRIPES_FOR_BARRED_AREA)
                {
                    results.push_back(tempResults[i]);
                }
            }
        }

        BarredArea::BarredArea()
        {}

        bool BarredArea::mergeAnotherBarredAreaStripe(std::shared_ptr<BarredArea> &otherBarredArea)
        {
            bool result = false;
            if((this->angleOfBarredArea - otherBarredArea->angleOfBarredArea) < 1.0)    // When angles are similar
            {
                // Determine whether current instance of barred area is bottom to the otherBarredArea
                bool isThisBarredAreaBottomSide = center.y > otherBarredArea->center.y;
                cv::Point centerOfTopStripeInBottomBarredArea, centerOfBottomStripeInTopBarredArea;
                if(isThisBarredAreaBottomSide)
                {
                    int numberOfStripes = stripes.size();
                    stripes[numberOfStripes - 1]->getCenterPoint(centerOfTopStripeInBottomBarredArea);
                    otherBarredArea->stripes[0]->getCenterPoint(centerOfBottomStripeInTopBarredArea);
                }
                else
                {
                    int numberOfStripes = otherBarredArea->stripes.size();
                    otherBarredArea->stripes[numberOfStripes - 1]->getCenterPoint(centerOfTopStripeInBottomBarredArea);
                    stripes[0]->getCenterPoint(centerOfBottomStripeInTopBarredArea);
                }

                float distanceBtwCenters = BarredAreaStripe::distanceBtwPoints(centerOfTopStripeInBottomBarredArea, centerOfBottomStripeInTopBarredArea);
                bool isValidDistance = false;
                isValidDistance = distanceBtwCenters <= MAX_DISTANCE_BETWEEN_STRIPES;
                isValidDistance = isValidDistance && (distanceBtwCenters >= MIN_DISTANCE_BETWEEN_STRIPES);
                if(isValidDistance)
                {
                    result = true;
                    if(isThisBarredAreaBottomSide)
                    {
                        for(int i = 0; i < otherBarredArea->stripes.size(); i++)
                        {
                            stripes.push_back(otherBarredArea->stripes[i]);
                        }
                    }
                    else
                    {
                        for(int i = 0; i < stripes.size(); i++)
                        {
                            otherBarredArea->stripes.push_back(stripes[i]);
                        }
                        stripes = otherBarredArea->stripes;
                    }
                    calculateCenter();
                }
            }
            return result;
        }

        void BarredArea::calculateCenter()
        {
            cv::Point bottomMostPoint, topMostPoint;
            std::shared_ptr<BarredAreaStripe> bottomMostStripe = stripes[0];
            std::shared_ptr<BarredAreaStripe> topMostStripe = stripes[stripes.size() - 1];
            std::array<cv::Point,4> barredAreaBox;
            bottomMostStripe->getBarredAreaStripe(barredAreaBox);
            bottomMostPoint.x = barredAreaBox[0].x;
            bottomMostPoint.y = barredAreaBox[0].y;
            topMostStripe->getBarredAreaStripe(barredAreaBox);
            topMostPoint.x = barredAreaBox[2].x;
            topMostPoint.y = barredAreaBox[2].y;

            center.x = (bottomMostPoint.x + topMostPoint.x) / 2;
            center.y = (bottomMostPoint.y + topMostPoint.y) / 2;
        }

        void BarredArea::getBarredAreaBox(std::array<cv::Point,4> &box)
        {
            std::array<cv::Point,4> barredAreaBox;
            getActualBarredAreaBox(barredAreaBox);

            // We have to extend our barred area dimensions to accomidate traingle stripe in actual barred area
            float distanceToOffset = (MIN_DISTANCE_BETWEEN_STRIPES + MIN_DISTANCE_BETWEEN_STRIPES) / 2;
            cv::Point2f unitVectorResult;

            // Offsetting bottom most point
            unitVector(barredAreaBox[3], barredAreaBox[0], unitVectorResult);
            box[0].x = barredAreaBox[0].x + (distanceToOffset * unitVectorResult.x);
            box[0].y = barredAreaBox[0].y + (distanceToOffset * unitVectorResult.y);

            // Offsetting point beside bottom most point which makes longest side
            unitVector(barredAreaBox[2], barredAreaBox[1], unitVectorResult);
            box[1].x = barredAreaBox[1].x + (distanceToOffset * unitVectorResult.x);
            box[1].y = barredAreaBox[1].y + (distanceToOffset * unitVectorResult.y);

            // Offsetting top most point
            unitVector(barredAreaBox[1], barredAreaBox[2], unitVectorResult);
            box[2].x = barredAreaBox[2].x + (distanceToOffset * unitVectorResult.x);
            box[2].y = barredAreaBox[2].y + (distanceToOffset * unitVectorResult.y);

            // Offsetting remainng point
            unitVector(barredAreaBox[0], barredAreaBox[3], unitVectorResult);
            box[3].x = barredAreaBox[3].x + (distanceToOffset * unitVectorResult.x);
            box[3].y = barredAreaBox[3].y + (distanceToOffset * unitVectorResult.y);
        }

        void BarredArea::unitVector(const cv::Point2f &startingPoint, const cv::Point2f &endingPoint, cv::Point2f &unitVectorResult)
        {
            float distanceBtwPoints = BarredAreaStripe::distanceBtwPoints(startingPoint, endingPoint);
            unitVectorResult.x = (endingPoint.x - startingPoint.x) / distanceBtwPoints;
            unitVectorResult.y = (endingPoint.y - startingPoint.y) / distanceBtwPoints;
        }

        void BarredArea::getActualBarredAreaBox(std::array<cv::Point,4> &box)
        {
            std::array<cv::Point,4> stripeBox;
            stripes[0]->getBarredAreaStripe(stripeBox);
            box[0].x = stripeBox[0].x;
            box[0].y = stripeBox[0].y;
            box[1].x = stripeBox[1].x;
            box[1].y = stripeBox[1].y;

            stripes[stripes.size() - 1]->getBarredAreaStripe(stripeBox);
            box[2].x = stripeBox[2].x;
            box[2].y = stripeBox[2].y;
            box[3].x = stripeBox[3].x;
            box[3].y = stripeBox[3].y;
        }

        void BarredArea::getCenter(cv::Point &centerResult)
        {
            centerResult.x = center.x;
            centerResult.y = center.y;
        }
    }
}