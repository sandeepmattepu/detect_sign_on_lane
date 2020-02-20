#include <iostream>
#include "ros/ros.h"
#include "detect_sign_on_lane/DetectSignsOnLane.h"

using namespace otto_car::lane_markings;

std::string nameOfLaneImage = "/perception/lane/debug/topview";
std::string locationOfSvmDat = "";
bool isDebugMode = true;

int parseCommandLineArguments(int argc, char** argv);

int main(int argc, char** argv)
{
	if(parseCommandLineArguments(argc, argv) == 1)
	{
		return 1;
	}

	ros::init(argc, argv, "detect_sign_on_lane");
	ros::NodeHandle nh;
	ROS_INFO("detect_signs_on_lane node started");

	std::string setDetectionFlagServiceName = "/set_sign_detection_on_lane_flag";
	std::unique_ptr<DetectSignsOnLane> detection(new DetectSignsOnLane(nh, locationOfSvmDat, nameOfLaneImage, 
													setDetectionFlagServiceName, isDebugMode));
	detection->init();
	while(ros::ok())
	{
		ros::spinOnce();
	}
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
			}
		}
		else if(argumentValue == "--debug" || argumentValue == "-d")
		{
			if(i + 1 < argc)
			{
				std::string debugMode = argv[i+1];
				if(debugMode == "true")
				{
					isDebugMode = true;
				}
			}
		}
	}
	if(locationOfSvmDat == "")
	{
		std::cout << "Usage rosrun detect_signs_on_lane detect_signs_on_lane -m path/to/svm/file -d true/false(for debug mode)" << std::endl;
		return 1;
	}
	return 0;
}