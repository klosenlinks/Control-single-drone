//geometry
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

//gazebo
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/GetModelState.h>

//ros
#include <ros/ros.h>

#include "stdio.h"//?
#include <std_msgs/Float64.h>//
#include <cmath>//
#include <tf2/LinearMath/Quaternion.h>//
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>//

class linkGazebo{

public:

	linkGazebo(const ros::NodeHandle& nh); //constructeur
	~linkGazebo(){};// destructeur
	void spinModel();

private:
	ros::NodeHandle nh;
	//URDF parameters
	std::string body_name;
	std::string reference_frame;
	geometry_msgs::Point referencePoint;

	//Output messages
	geometry_msgs::Vector3 force;
	geometry_msgs::Vector3 torque;
	geometry_msgs::Wrench wrench;	

	//Input messages
	geometry_msgs::PoseStamped poseMsgOut;
	geometry_msgs::TwistStamped twistMsgOut;

	gazebo_msgs::ApplyBodyWrench applyBodyWrench;
	gazebo_msgs::GetModelState getModelState;

	float thrustMsgIn;
	float tauxMsgIn;
	float tauyMsgIn;
	float tauzMsgIn;

	//Services
	ros::ServiceClient applyBodyWrenchClient;
	ros::ServiceClient getModelStateClient;

	//functions
	void sendForce();
	void sendModelState();
	
	//Publishers 
	ros::Publisher posePub;
	ros::Publisher twistPub;

	//Subscribers
	ros::Subscriber thrustSub;
	ros::Subscriber tauxSub;
	ros::Subscriber tauySub;
	ros::Subscriber tauzSub;	
	
	//Callbacks
	void thrustCallBack(const std_msgs::Float64::ConstPtr& msg);
	void tauxCallBack(const std_msgs::Float64::ConstPtr& msg);
	void tauyCallBack(const std_msgs::Float64::ConstPtr& msg);
	void tauzCallBack(const std_msgs::Float64::ConstPtr& msg);

	//Quaternions
        float q0=1.0;
        float q1=0;
        float q2=0;
        float q3=0;

	//Rotation matrix
	//We only need the last column (ax,ay,az) here
	float ax = 0;
	float ay = 0;
	float az = 0;

};
