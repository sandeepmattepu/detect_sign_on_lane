#include "detect_sign_on_lane/DetectSignsOnLane.h"

namespace otto_car
{
	namespace lane_markings
	{
	
	DetectSignsOnLane::DetectSignsOnLane(ros::NodeHandle &nh)
	{
		this->setDetectionFlagService = nh.advertiseService(this->setDetectionFlagServiceName, 
																&DetectSignsOnLane::setDetectionFlagServiceCallBack, this);
	}

	bool DetectSignsOnLane::setDetectionFlagServiceCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
	{
		this->continueDetection = req.data;

		res.success = this->continueDetection;
		if(res.success)
		{
			res.message = "Detection of signs on the lane has started";
			ROS_INFO("Detection of signs on the lane has started");
		}
		else
		{
			res.message = "Detection of signs on the lane is paused/stopped";
			ROS_INFO("Detection of signs on the lane is paused/stopped");
		}
		return true;
	}
	
	}
}