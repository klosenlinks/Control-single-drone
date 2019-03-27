//ROS
#include <ros/ros.h>

//Messages
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


class trajectory{

public :
	
	trajectory(const ros::NodeHandle& nh);
	~trajectory(){};
	void spin();
	void generateTrajectory();

private:
	
	ros::NodeHandle nh; 

	//Messages
	geometry_msgs::PoseArray pose_array;
	geometry_msgs::Pose pose_element;
	geometry_msgs::PoseStamped desiredPoseMsgOut;

	//Publishers
	ros::Publisher desiredPosePub;

	//Counter 
	int cpt=0;

	//Other functions
	void sendDesiredPose(int cpt);
};
