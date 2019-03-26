//ROS
#include <ros/ros.h>

//Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

//Gazebo
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/GetModelState.h>

class linkGazebo{

public:

	linkGazebo(const ros::NodeHandle& nh); 
	~linkGazebo(){};
	void spinModel(int i);

private:
	ros::NodeHandle nh;

	//URDF parameters
	std::string body_name;
	std::string reference_frame;
	geometry_msgs::Point referencePoint;

	//Input messages
	float thrustMsgIn;
	float tauxMsgIn;
	float tauyMsgIn;
	float tauzMsgIn;

	//Output messages
	geometry_msgs::PoseStamped poseMsgOut;
	geometry_msgs::TwistStamped twistMsgOut;
	
	//Gazebo messages
	gazebo_msgs::ApplyBodyWrench applyBodyWrench;
	gazebo_msgs::GetModelState getModelState;

	geometry_msgs::Vector3 force;
	geometry_msgs::Vector3 torque;
	geometry_msgs::Wrench wrench;

        geometry_msgs::Vector3 perturbation;

	//Services
	ros::ServiceClient applyBodyWrenchClient;
	ros::ServiceClient getModelStateClient;
	
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

	//Quaternion
        float q0=1.0;
        float q1=0;
        float q2=0;
        float q3=0;

	//Rotation matrix
	//We only need the last column (ax,ay,az) here
	float ax = 0;
	float ay = 0;
	float az = 0;

	//Other functions
	void sendForce(int i);
	void sendModelState();
};
