#include "ros/ros.h"
#include "stdio.h"
#include <geometry_msgs/PoseStamped.h>



int main(int argc, char**argv)
{
        ros::init(argc, argv, "setpoint_publisher");
        ros::NodeHandle nh;
	ros::Publisher desiredPosepub;
	desiredPosepub = nh.advertise<geometry_msgs::PoseStamped>("/desiredpose",2);

        ros::Rate Rate(200);
	
	geometry_msgs::PoseStamped desiredPose;
	
        while(ros::ok)
        {
		desiredPose.pose.position.x = 20;
		desiredPose.pose.position.y = 40;
		desiredPose.pose.position.z = 60;
		
		desiredPosepub.publish(desiredPose);
                Rate.sleep();
        }
}

