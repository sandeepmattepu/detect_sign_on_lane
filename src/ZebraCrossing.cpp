#include "detect_sign_on_lane/ZebraCrossing.h"

namespace otto_car
{
	namespace lane_markings
	{
        ZebraCrossing::ZebraCrossing()
        {
            for(int i = 0; i < boundingBox.size(); i++)
            {
                boundingBox[i].x = 0;
                boundingBox[i].y = 0;
            }
            calculateDimensionsOfZebraCrossing();
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

        void ZebraCrossing::clusterStripesForZebraCrossings(std::vector<std::shared_ptr<cv::RotatedRect>> &stripes,
                            std::vector<std::shared_ptr<ZebraCrossing>> &results)
        {
            if(stripes.size() == 0)
            {
                return;
            }
            
            std::vector<std::shared_ptr<ZebraCrossing>> tempResults;
            for(int i = 0; i < stripes.size(); i++)     // Treat each stripe as a zebra crossing
            {
                std::shared_ptr<ZebraCrossing> tempZebraCrossing = std::make_shared<ZebraCrossing>();
                std::shared_ptr<cv::Point2f> stripeCenter = std::make_shared<cv::Point2f>();
                stripeCenter->x = stripes[i]->center.x;
                stripeCenter->y = stripes[i]->center.y;
                tempZebraCrossing->stripeCenters.push_back(stripeCenter);
                tempZebraCrossing->angleOfZebraCrossing = stripes[i]->angle;
                tempZebraCrossing->widthOfZebraCrossing = stripes[i]->size.width;
                tempZebraCrossing->heightOfZebraCrossing = stripes[i]->size.height;
                tempZebraCrossing->centerOfZebraCrossing.x = stripes[i]->center.x;
                tempZebraCrossing->centerOfZebraCrossing.y = stripes[i]->center.y;

                cv::Point2f vertices[4];
				stripes[i]->points(vertices);

                // Stripes which has width greater than height has wrong positions of rotated rect. Following code is to fix that
                if(tempZebraCrossing->widthOfZebraCrossing > tempZebraCrossing->heightOfZebraCrossing)
                {
                    float tempDimension = tempZebraCrossing->widthOfZebraCrossing;
                    tempZebraCrossing->widthOfZebraCrossing = tempZebraCrossing->heightOfZebraCrossing;
                    tempZebraCrossing->heightOfZebraCrossing = tempDimension;

                    // Bottom left of stripe is top left in rotated rect
                    tempZebraCrossing->boundingBox[0].x = vertices[1].x;
                    tempZebraCrossing->boundingBox[0].y = vertices[1].y;

                    // Top left of stripe is top right in rotated rect
                    tempZebraCrossing->boundingBox[1].x = vertices[2].x;
                    tempZebraCrossing->boundingBox[1].y = vertices[2].y;

                    // Top right of stripe is bottom right in rotated rect
                    tempZebraCrossing->boundingBox[2].x = vertices[3].x;
                    tempZebraCrossing->boundingBox[2].y = vertices[3].y;

                    // Bottom right of stripe is bottom left in rotated rect
                    tempZebraCrossing->boundingBox[3].x = vertices[0].x;
                    tempZebraCrossing->boundingBox[3].y = vertices[0].y;
                }
                else        // -ve angle has proper position for rotated rect
                {
                    for(int j = 0; j < 4; j++)
                    {
                        tempZebraCrossing->boundingBox[j].x = vertices[j].x;
                        tempZebraCrossing->boundingBox[j].y = vertices[j].y;
                    } 
                }
                tempResults.push_back(tempZebraCrossing);
            }

            /* 1. Zebra crossing stipes are combined together when distance between them is appropriate.
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
                std::shared_ptr<ZebraCrossing> stripeA = tempResults[i];
                int numberOfMerges = 1;
                while(numberOfMerges > 0)
                {
                    numberOfMerges = 0;
                    for(int j = (i + 1); j < tempResults.size(); j++)
                    {
                        std::shared_ptr<ZebraCrossing> stripeB = tempResults[j];
                        if(stripeA->mergeAnotherZebraCrossing(stripeB))
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
                if(tempResults[i]->stripeCenters.size() >= MIN_STRIPES_FOR_ZEBRA_CROSSING)
                {
                    results.push_back(tempResults[i]);
                }
            }
        }

        bool ZebraCrossing::mergeAnotherZebraCrossing(std::shared_ptr<ZebraCrossing> otherZebraCrossing)
        {
            bool result = false;
            if((this->angleOfZebraCrossing - otherZebraCrossing->angleOfZebraCrossing) < 1.0)   // When angles are similar
            {
                // Determine whether current instance of zebra crossing is towards left of the otherZebraCrossing
                bool isThisZebraCrossingLeftSide = centerOfZebraCrossing.x < otherZebraCrossing->centerOfZebraCrossing.x;
                cv::Point2f centerOfRightStripeInLeftZebCros, centerOfLeftStripeInRightZebCros;
                if(isThisZebraCrossingLeftSide)
                {
                    int numStripesInLeftZebCros = stripeCenters.size();
                    centerOfRightStripeInLeftZebCros.x = stripeCenters[numStripesInLeftZebCros - 1]->x;
                    centerOfRightStripeInLeftZebCros.y = stripeCenters[numStripesInLeftZebCros - 1]->y;
                    centerOfLeftStripeInRightZebCros.x = otherZebraCrossing->stripeCenters[0]->x;
                    centerOfLeftStripeInRightZebCros.y = otherZebraCrossing->stripeCenters[0]->y;
                }
                else
                {
                    int numStripesInLeftZebCros = otherZebraCrossing->stripeCenters.size();
                    centerOfRightStripeInLeftZebCros.x = otherZebraCrossing->stripeCenters[numStripesInLeftZebCros - 1]->x;
                    centerOfRightStripeInLeftZebCros.y = otherZebraCrossing->stripeCenters[numStripesInLeftZebCros - 1]->y;
                    centerOfLeftStripeInRightZebCros.x = stripeCenters[0]->x;
                    centerOfLeftStripeInRightZebCros.y = stripeCenters[0]->y;
                }
                
                // Finding distance between 'centerOfRightStripeInLeftZebCros' and 'centerOfLeftStripeInRightZebCros'
                float xComponent = (centerOfRightStripeInLeftZebCros.x - centerOfLeftStripeInRightZebCros.x) * 
                                   (centerOfRightStripeInLeftZebCros.x - centerOfLeftStripeInRightZebCros.x);
                float yComponent = (centerOfRightStripeInLeftZebCros.y - centerOfLeftStripeInRightZebCros.y) * 
                                   (centerOfRightStripeInLeftZebCros.y - centerOfLeftStripeInRightZebCros.y);
                float distanceBtwStripes = sqrt(xComponent + yComponent);

                bool isValidDistance = false;
                isValidDistance = distanceBtwStripes <= MAX_DISTANCE_BTW_ZEBRA_STRIPES;
                isValidDistance = isValidDistance && (distanceBtwStripes >= MIN_DISTANCE_BTW_ZEBRA_STRIPES);
                if(isValidDistance)
                {
                    result = true;
                    // Adding stripes(i.e stripe centers) to current instance and calculating new bounding box
                    if(isThisZebraCrossingLeftSide)
                    {
                        for(int i = 0; i < otherZebraCrossing->stripeCenters.size(); i++)
                        {
                            stripeCenters.push_back(otherZebraCrossing->stripeCenters[i]);
                        }
                        // Top right has to be changed
                        boundingBox[2].x = otherZebraCrossing->boundingBox[2].x;
                        boundingBox[2].y = otherZebraCrossing->boundingBox[2].y;
                        // Bottom right has to be changed
                        boundingBox[3].x = otherZebraCrossing->boundingBox[3].x;
                        boundingBox[3].y = otherZebraCrossing->boundingBox[3].y;
                    }
                    else
                    {
                        for(int i = 0; i < stripeCenters.size(); i++)
                        {
                            otherZebraCrossing->stripeCenters.push_back(stripeCenters[i]);
                        }
                        stripeCenters = otherZebraCrossing->stripeCenters;

                        // Bottom left has to be changed
                        boundingBox[0].x = otherZebraCrossing->boundingBox[0].x;
                        boundingBox[0].y = otherZebraCrossing->boundingBox[0].y;
                        // Top left has to be changed
                        boundingBox[1].x = otherZebraCrossing->boundingBox[1].x;
                        boundingBox[1].y = otherZebraCrossing->boundingBox[1].y;
                    }
                    calculateDimensionsOfZebraCrossing();
                }
            }
            return result;
        }

        void ZebraCrossing::calculateDimensionsOfZebraCrossing()
        {
            centerOfZebraCrossing.x = (boundingBox[0].x + boundingBox[2].x) / 2;
            centerOfZebraCrossing.y = (boundingBox[0].y + boundingBox[2].y) / 2;

            float xComponent = (boundingBox[0].x - boundingBox[3].x) * (boundingBox[0].x - boundingBox[3].x);
            float yComponent = (boundingBox[0].y - boundingBox[3].y) * (boundingBox[0].y - boundingBox[3].y);
            widthOfZebraCrossing = sqrt(xComponent + yComponent);

            xComponent = (boundingBox[1].x - boundingBox[2].x) * (boundingBox[1].x - boundingBox[2].x);
            yComponent = (boundingBox[1].y - boundingBox[2].y) * (boundingBox[1].y - boundingBox[2].y);
            heightOfZebraCrossing = sqrt(xComponent + yComponent);
        }

        void ZebraCrossing::boundingBoxOfZebraCrossing(std::array<cv::Point2f,4> &results)
        {
            for(int i = 0; i < results.size(); i++)
            {
                results[i].x = boundingBox[i].x;
                results[i].y = boundingBox[i].y;
            }
        }

        float ZebraCrossing::getAngleOfZebraCrossing()
        {
            return angleOfZebraCrossing;
        }

        float ZebraCrossing::getWidthOfZebraCrossing()
        {
            return widthOfZebraCrossing;
        }

        float ZebraCrossing::getHeightOfZebraCrossing()
        {
            return heightOfZebraCrossing;
        }

        void ZebraCrossing::getCenterOfZebraCrossing(cv::Point2f &result)
        {
            result.x = centerOfZebraCrossing.x;
            result.y = centerOfZebraCrossing.y;
        }

        int ZebraCrossing::getNumberOfStripes()
        {
            stripeCenters.size();
        }
    }
}