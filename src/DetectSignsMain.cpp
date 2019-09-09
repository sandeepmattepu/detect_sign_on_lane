#include <iostream>
#include "ros/ros.h"
#include "detect_sign_on_lane/DetectSignsOnLane.h"

using namespace otto_car::lane_markings;

std::string nameOfLaneImage = "/perception/lane/debug/topview";
std::string locationOfSvmDat;

int parseCommandLineArguments(int argc, char** argv);

int main(int argc, char** argv)
{
	if(parseCommandLineArguments(argc, argv) == 1)
	{
		return 1;
	}

	ros::init(argc, argv, "detect_signs_on_lane");
	ros::NodeHandle nh;
	ROS_INFO("detect_signs_on_lane node started");

	std::string setDetectionFlagServiceName = "/set_sign_detection_on_lane_flag";
	std::unique_ptr<DetectSignsOnLane> detection(new DetectSignsOnLane(nh, locationOfSvmDat, nameOfLaneImage, 
													setDetectionFlagServiceName));
	ros::spin();

	return 0;
}

int parseCommandLineArguments(int argc, char** argv)
{
	for(int i = 0; i < argc; i++)
	{
		std::string argumentValue = std::string(argv[i]);
		if(argumentValue == "--model" || argumentValue == "-m")
		{
			if( i + 1 < argc)
			{
				locationOfSvmDat = argv[i+1];
				return 0;
			}
		}
	}
	std::cout << "Please provide location of SVM model file" << std::endl;
	return 1;
}