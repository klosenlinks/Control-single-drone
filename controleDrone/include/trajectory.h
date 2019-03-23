#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <cmath>

class trajectory{

public :
	
	trajectory(const ros::NodeHandle& nh);
	~trajectory(){};
	void spin();
	void generateTrajectory();

private:
	
	ros::NodeHandle nh;

	double epsilon = 0.1; // Erreur admissible par rapport aux waypoints
	
	//callbacks
	void desiredPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);
	//messages
	geometry_msgs::PoseArray pose_array;
	geometry_msgs::Pose pose_element;
	geometry_msgs::PoseStamped desiredPoseMsgIn;
	geometry_msgs::PoseStamped desiredPoseMsgOut;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::Pose error;

	//subscribers
	ros::Subscriber desiredPoseSub;
	ros::Subscriber poseSub;

	//publishers
	ros::Publisher desiredPosePub;

	//compteur
	int cpt=0;

	//functions
	void sendDesiredPose(int cpt);
};

double norm(const geometry_msgs::Pose& p);
