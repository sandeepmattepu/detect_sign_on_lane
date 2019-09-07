#include <iostream>
#include "ros/ros.h"
#include "detect_sign_on_lane/DetectSignsOnLane.h"

using namespace otto_car::lane_markings;

std::string nameOfLaneImage = "/perception/lane/debug/topview";

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		std::cout << "Pass the location of the SVM DAT file" << std::endl;
		return 1;
	}

	std::string locationOfSvmDat = argv[1];
	ros::init(argc, argv, "detect_signs_on_lane");
	ros::NodeHandle nh;
	ROS_INFO("detect_signs_on_lane node started");

	std::string setDetectionFlagServiceName = "/set_sign_detection_on_lane_flag";
	std::unique_ptr<DetectSignsOnLane> detection(new DetectSignsOnLane(nh, locationOfSvmDat, nameOfLaneImage, 
													setDetectionFlagServiceName));
	ros::spin();

	return 0;
}