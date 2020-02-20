#include <iostream>
#include <thread>
#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

cv::Mat originalImage;
int64_t lastCallBackTime = 0;
bool isCloseApplication = false;
std::string nameOfTopic = "";

void callBack(const sensor_msgs::Image &image);
void inspectRotatedRect(cv::Mat &originalImg);
void distanceBtwContours(cv::Mat &originalImg);
void inspectBarredAreaStripes(cv::Mat &originalImg);
void showGUI();

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cout << "Usage " << argv[0] << " [Name of the topic]" << std::endl;
        return 1;
    }
    ros::init(argc, argv, "inspect_contours");
    ros::NodeHandle nodeHandle;
    nameOfTopic = argv[1];
    ros::Subscriber subscribe = nodeHandle.subscribe(nameOfTopic, 40, callBack);
    std::thread guiThread(showGUI);
    while(ros::ok() && !isCloseApplication)
    {
        ros::spinOnce();
    }
    guiThread.join();
    return 0;
}

void callBack(const sensor_msgs::Image &image)
{
    lastCallBackTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy(image, "bgr8");
    originalImage = imagePtr->image.clone();
}

void showGUI()
{
    cv::namedWindow("Inspection mode");
    while (!isCloseApplication)
    {
        int64_t currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if(currentTime - lastCallBackTime > (1000))
        {
            cv::Mat blankImage = cv::Mat::zeros(540,600,CV_64F);
            cv::putText(blankImage, "Topic is not being published", cv::Point2d(20,300), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(255, 0, 0));
            cv::putText(blankImage, "Press escape to close the window.", cv::Point2d(20,520), cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(255,0,0));
            cv::imshow("Inspection mode", blankImage);
            int keyPressed = cv::waitKey(1000/45);
            if(keyPressed == 27)
            {
                isCloseApplication = true;
            }
            continue;
        }
        cv::Mat image = originalImage.clone();
        cv::putText(image, "Press space to inspect a frame.", cv::Point2d(20,500), cv::FONT_HERSHEY_SIMPLEX, 0.5,
        cv::Scalar(0,0,255));
        cv::putText(image, "Press escape to close the window.", cv::Point2d(20,520), cv::FONT_HERSHEY_SIMPLEX, 0.5,
        cv::Scalar(0,0,255));
        cv::imshow("Inspection mode", image);
        int keyPressed = cv::waitKey(1000/45);
        if(keyPressed == 27)
        {
            isCloseApplication = true;
        }
        else if(keyPressed == 32)
        {
            cv::Mat capturedFrame = originalImage.clone();
            while (true)
            {
                cv::Mat image1 = capturedFrame.clone();
                cv::putText(image1, "Choose following kind of inspection", cv::Point2d(20,420), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0,0,255));
                cv::putText(image1, "[1] Width, height, angle of rotated rect of contour", cv::Point2d(20,440), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0,0,255));
                cv::putText(image1, "[2] Distance between contours", cv::Point2d(20,460), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0,0,255));
                cv::putText(image1, "[3] Inspection of barred area stripes", cv::Point2d(20,480), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0,0,255));
                cv::putText(image1, "Press escape to go to see the live topic msg", cv::Point2d(20,500), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0,0,255));
                cv::imshow("Inspection mode", image1);
                int inspectionMode = cv::waitKey(0);
                if(inspectionMode == 49)
                {
                    inspectRotatedRect(capturedFrame);
                }
                else if(inspectionMode == 50)
                {
                    distanceBtwContours(capturedFrame);
                }
                else if(inspectionMode == 51)
                {
                    inspectBarredAreaStripes(capturedFrame);
                }
                else if(inspectionMode == 27)
                {
                    break;
                }
            }
        }
    }
    cv::destroyAllWindows();
}


void inspectRotatedRect(cv::Mat &originalImg)
{
    cv::Mat image = originalImg.clone();
    cv::Mat greyImage, thresholdImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);
    cv::threshold(greyImage, thresholdImage, 90, 255, cv::THRESH_BINARY_INV);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(thresholdImage.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    int contourIndex = 0;
    bool exitFunction = false;
    while(!exitFunction)
    {
        while (true)
        {
            cv::RotatedRect rotatedRect = cv::minAreaRect(contours[contourIndex]);
            image = originalImg.clone();
            cv::Point2f vertices[4];
            rotatedRect.points(vertices);
            for(int i = 0; i < 4; i++)
            {
                cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0), 2);
            }
            std::string contourNumberMsg = "Highlighting contour " + std::to_string(contourIndex+1) + " out of " + std::to_string(contours.size());
            cv::putText(image, contourNumberMsg, cv::Point2d(20,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));
            cv::putText(image, "Press space to inspect the contour", cv::Point2d(20,460), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0,0,255));
            cv::putText(image, "Press 'N' to skip to the next contour", cv::Point2d(20,480), cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(0,0,255));
            cv::putText(image, "Press 'P' for previous contour", cv::Point2d(20,500), cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(0,0,255));
            cv::putText(image, "Press escape to see inspection options", cv::Point2d(20,520), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0,0,255));
            cv::imshow("Inspection mode", image);
            int buttonPressed = cv::waitKey(0);
            if(buttonPressed == 32)
            {
                std::cout << "Angle of rect is " << rotatedRect.angle << std::endl;
                std::cout << "Height of rect is " << rotatedRect.size.height << std::endl;
                std::cout << "Width of rect is " << rotatedRect.size.width << std::endl;
                if(rotatedRect.size.width > rotatedRect.size.height)
                {
                    std::cout << "Height and width values might be swapped in the code" << std::endl;
                }
                std::cout << "Center x is " << rotatedRect.center.x << " y is " << rotatedRect.center.y << std::endl;
            }
            else if(buttonPressed == 80 || buttonPressed == 112)
            {
                contourIndex -= 1;
                contourIndex = contourIndex < 0 ? (contours.size() - 1) : contourIndex;
            }
            else if(buttonPressed == 27)
            {
                exitFunction = true;
                break;
            }
            else if(buttonPressed == 78 || buttonPressed == 110)
            {
                break;
            }
        }
        contourIndex += 1;
        contourIndex = contourIndex % contours.size();
    }
}

void distanceBtwContours(cv::Mat &originalImg)
{
    bool firstRectSelected = false;
    cv::RotatedRect firstRectBox;
    cv::Mat image = originalImg.clone();
    cv::Mat greyImage, thresholdImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);
    cv::threshold(greyImage, thresholdImage, 90, 255, cv::THRESH_BINARY_INV);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(thresholdImage.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    int contourIndex = 0;
    bool exitFunction = false;
    cv::Mat imageWithFirstContourHighlighted;
    while(!exitFunction)
    {
        while (true)
        {
            cv::RotatedRect rotatedRect = cv::minAreaRect(contours[contourIndex]);
            if(firstRectSelected)
            {
                image = imageWithFirstContourHighlighted.clone();
            }
            else
            {
                image = originalImg.clone();
            }
            cv::Point2f vertices[4];
            rotatedRect.points(vertices);
            for(int i = 0; i < 4; i++)
            {
                cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0), 2);
            }
            cv::Mat contoursHighlighted = image.clone();
            std::string topCornerMsg = "";
            if(firstRectSelected)
            {
                topCornerMsg = "Select second contour to measure distance with first contour";
                cv::line(image, firstRectBox.center, rotatedRect.center, cv::Scalar(0,0,255),2);
            }
            else
            {
                topCornerMsg = "Select first contour for measuring distance";
            }
            cv::putText(image, topCornerMsg, cv::Point2d(20,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));
            cv::putText(image, "Press space to select", cv::Point2d(20,480), cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(0,0,255));
            cv::putText(image, "Press escape to see inspection options", cv::Point2d(20,500), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0,0,255));
            cv::putText(image, "Press 'N' to skip to next contour", cv::Point2d(20,520), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0,0,255));
            cv::imshow("Inspection mode", image);
            int buttonPressed = cv::waitKey(0);
            if(buttonPressed == 32)
            {
                if(firstRectSelected)
                {
                    double xElement = (rotatedRect.center.x - firstRectBox.center.x) * (rotatedRect.center.x - firstRectBox.center.x);
                    double yElement = (rotatedRect.center.y - firstRectBox.center.y) * (rotatedRect.center.y - firstRectBox.center.y);
                    double distance = sqrt(xElement + yElement);
                    std::cout << "Distance between contours is " << distance << std::endl;
                    firstRectSelected = false;
                }
                else
                {
                    firstRectSelected = true;
                    imageWithFirstContourHighlighted = contoursHighlighted.clone();
                    firstRectBox = rotatedRect;
                }
                break;   
            }
            else if(buttonPressed == 27)
            {
                exitFunction = true;
                break;
            }
            else if(buttonPressed == 78 || buttonPressed == 110)
            {
                break;
            }
        }
        
        contourIndex += 1;
        contourIndex %= contours.size();
    }
}

void inspectBarredAreaStripes(cv::Mat &originalImg)
{
    cv::Mat image = originalImg.clone();
    cv::Mat greyImage, thresholdImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);
    cv::threshold(greyImage, thresholdImage, 90, 255, cv::THRESH_BINARY_INV);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(thresholdImage.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    int contourIndex = 0;
    bool exitFunction = false;
    while(!exitFunction)
    {
        while (true)
        {
            cv::RotatedRect rotatedRect = cv::minAreaRect(contours[contourIndex]);
            image = originalImg.clone();
            cv::Point2f vertices[4];
            rotatedRect.points(vertices);
            for(int i = 0; i < 4; i++)
            {
                cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0), 2);
            }
            std::string contourNumberMsg = "Highlighting contour " + std::to_string(contourIndex+1) + " out of " + std::to_string(contours.size());
            cv::putText(image, contourNumberMsg, cv::Point2d(20,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));
            cv::putText(image, "Press space to inspect the contour", cv::Point2d(20,460), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0,0,255));
            cv::putText(image, "Press 'N' to skip to the next contour", cv::Point2d(20,480), cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(0,0,255));
            cv::putText(image, "Press 'P' for previous contour", cv::Point2d(20,500), cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(0,0,255));
            cv::putText(image, "Press escape to see inspection options", cv::Point2d(20,520), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0,0,255));
            cv::imshow("Inspection mode", image);
            int buttonPressed = cv::waitKey(0);
            if(buttonPressed == 32)
            {
                // Check whether approximated contour has four sides
                float epsilonCoEff = 0.05;
                double arcLength = cv::arcLength(contours[contourIndex], true);
                std::vector<cv::Point> approxPolyPoints;
                cv::approxPolyDP(contours[contourIndex], approxPolyPoints, epsilonCoEff * arcLength, true);
                if(approxPolyPoints.size() == 4)
                {
                    for(int i = 0; i < approxPolyPoints.size(); i++)
                    {
                        double xElement = (approxPolyPoints[i].x - approxPolyPoints[(i+1)%4].x) * (approxPolyPoints[i].x - approxPolyPoints[(i+1)%4].x);
                        double yElement = (approxPolyPoints[i].y - approxPolyPoints[(i+1)%4].y) * (approxPolyPoints[i].y - approxPolyPoints[(i+1)%4].y);
                        double distance = sqrt(xElement + yElement);
                        int secondPoint = (i+2);
                        secondPoint = secondPoint == 5 ? 1 : secondPoint;
                        std::cout << "Length of side with point " << (i + 1) << " and " << secondPoint << " is " << distance << std::endl;
                        int previousPointIndex = i - 1;
                        previousPointIndex = previousPointIndex < 0 ? (approxPolyPoints.size() - 1) : previousPointIndex;
                        int nextPointIndex = i + 1;
                        nextPointIndex %= approxPolyPoints.size();
                        float vec1XComponent = approxPolyPoints[previousPointIndex].x - approxPolyPoints[i].x;
                        float vec1YComponent = approxPolyPoints[previousPointIndex].y - approxPolyPoints[i].y;
                        float vec2XComponent = approxPolyPoints[i].x - approxPolyPoints[nextPointIndex].x;
                        float vec2YComponent = approxPolyPoints[i].y - approxPolyPoints[nextPointIndex].y;

                        float vec1Magnitude = sqrt((vec1XComponent  * vec1XComponent) + (vec1YComponent * vec1YComponent));
                        float vec2Magnitude = sqrt((vec2XComponent * vec2XComponent) + (vec2YComponent * vec2YComponent));

                        float dotProduct = (vec1XComponent * vec2XComponent) + (vec1YComponent * vec2YComponent);

                        float angleInRadians = acos(dotProduct / (vec1Magnitude * vec2Magnitude));
                        std::cout << "Angle at point " << (i + 1) << " is " << (angleInRadians * 180 / 3.141592653589793238463) << std::endl;
                    }
                }
                else
                {
                    std::cout << "The selected contour cannot approximately have a 4 sided polygon" << std::endl;
                }
            }
            else if(buttonPressed == 80 || buttonPressed == 112)
            {
                contourIndex -= 1;
                contourIndex = contourIndex < 0 ? (contours.size() - 1) : contourIndex;
            }
            else if(buttonPressed == 27)
            {
                exitFunction = true;
                break;
            }
            else if(buttonPressed == 78 || buttonPressed == 110)
            {
                break;
            }
        }
        contourIndex += 1;
        contourIndex = contourIndex % contours.size();
    }
}