//geometry
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
//#include <duration.h>
//gazebo
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/GetModelState.h>

#include <ros/ros.h>
#include "stdio.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
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

	//Outputs
	geometry_msgs::Vector3 force;
	geometry_msgs::Vector3 torque;
	geometry_msgs::Wrench wrench;	

	//messages
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
	
	//Publishers and subscribers
	ros::Publisher posePub;
	ros::Publisher twistPub;

	ros::Subscriber thrustSub;
	ros::Subscriber tauxSub;
	ros::Subscriber tauySub;
	ros::Subscriber tauzSub;	

	
	//Callbacks
	void thrustCallBack(const std_msgs::Float64::ConstPtr& msg);
	void tauxCallBack(const std_msgs::Float64::ConstPtr& msg);
	void tauyCallBack(const std_msgs::Float64::ConstPtr& msg);
	void tauzCallBack(const std_msgs::Float64::ConstPtr& msg);

};
