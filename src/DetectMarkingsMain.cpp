#include "ros/ros.h"
#include "detect_sign_on_lane/DetectMarkingsOnLane.h"

using namespace otto_car::lane_markings;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "detect_markings_on_lane");
	ros::NodeHandle nh;
	ROS_INFO("Detection of markers on the lane node started");

	std::unique_ptr<DetectMarkingsOnLane> detection(new DetectMarkingsOnLane(nh));
	ros::spin();

	return 0;
}