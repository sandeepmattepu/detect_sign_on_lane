#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <string>

namespace otto_car
{
	namespace lane_markings
	{
		class DetectSignsOnLane
		{
			private:
				bool continueDetection = false;

				std::string setDetectionFlagServiceName = "/set_sign_detection_on_lane_flag";
				ros::ServiceServer setDetectionFlagService;

			public:
				DetectSignsOnLane(ros::NodeHandle &nh);
				bool setDetectionFlagServiceCallBack(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
		};
	}
}