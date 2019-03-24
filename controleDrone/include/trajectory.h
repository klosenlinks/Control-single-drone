//ROS
#include <ros/ros.h>

//Messages
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

//Math
#include <cmath>

class trajectory{

public :
	
	trajectory(const ros::NodeHandle& nh);
	~trajectory(){};
	void spin();
	void generateTrajectory();

private:
	
	ros::NodeHandle nh;

	//Allowable error
	double epsilon = 0.1; 
	
	//Callbacks
	void desiredPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);

	//Messages
	geometry_msgs::PoseArray pose_array;
	geometry_msgs::Pose pose_element;
	geometry_msgs::PoseStamped desiredPoseMsgIn;
	geometry_msgs::PoseStamped desiredPoseMsgOut;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::Pose error;

	//Subscribers
	ros::Subscriber desiredPoseSub;
	ros::Subscriber poseSub;

	//Publishers
	ros::Publisher desiredPosePub;

	//Counter 
	int cpt=0;

	//Other functions
	void sendDesiredPose(int cpt);
};

double norm(const geometry_msgs::Pose& p);
