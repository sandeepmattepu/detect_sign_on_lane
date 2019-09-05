#include "ros/ros.h"
#include "detect_sign_on_lane/DetectSignsOnLane.h"

using namespace otto_car::lane_markings;

std::string nameOfLaneImage = "/perception/lane/debug/topview";

int main(int argc, char** argv)
{
	ros::init(argc, argv, "detect_signs_on_lane");
	ros::NodeHandle nh;
	ROS_INFO("detect_signs_on_lane node started");

	std::unique_ptr<DetectSignsOnLane> detection(new DetectSignsOnLane(nh, nameOfLaneImage));
	ros::spin();

	return 0;
}