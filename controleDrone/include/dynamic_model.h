#include <ros/ros.h>
#include "stdio.h"
//#include "Eigen/Eigen"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class dynamicModel{
public:

	dynamicModel(const ros::NodeHandle& n,float m, float ixx, float iyy, float izz); //constructeur
	~dynamicModel(){};// destructeur
	void spinModel();

private:
	
	//parameters
	ros::NodeHandle nh;
	float mass;
	float ixx;
	float iyy;
	float izz;

	//Inputs
	float thrust = 0;
	float taux = 0;
	float tauy = 0;
	float tauz = 0;
	
	//Variables and initial values
	float x=0;
	float xd=0;
	float xdd=0;

	float y=0;
	float yd=0;
	float ydd=0;

	float z=0;
	float zd=0;
	float zdd=0;

	float phi=0;
	float phid=0;
	float p=0;
	float pd=0;

	float theta=0;
	float thetad=0;
	float q=0;
	float qd=0;

	float psi=0;
	float psid=0;
	float r=0;
	float rd=0;
	
	//previous values stored for integration	
	float x_=0;
	float xd_=0;
	float xdd_=0;

	float y_=0;
	float yd_=0;
	float ydd_=0;

	float z_=0;
	float zd_=0;
	float zdd_=0;

	float phi_=0;
	float phid_=0;
	float p_=0;
	float pd_=0;

	float theta_=0;
	float thetad_=0;
	float q_=0;
	float qd_=0;

	float psi_=0;
	float psid_=0;
	float r_=0;
	float rd_=0;
	
	//Rotation matrix
	//We only need the last column (ax,ay,az) here
	float ax = 0;
	float ay = 0;
	float az = 0;
	
	//Orientation
	tf2::Quaternion orientation;	

	//Publishers and subscribers
	ros::Publisher posePub;
	ros::Publisher twistPub;

	ros::Subscriber thrustSub;
	ros::Subscriber tauxSub;
	ros::Subscriber tauySub;
	ros::Subscriber tauzSub;	

	//messages
	geometry_msgs::PoseStamped poseMsgOut;
	geometry_msgs::TwistStamped twistMsgOut;

	std_msgs::Float64 thrustMsgIn;
	std_msgs::Float64 tauxMsgIn;
	std_msgs::Float64 tauyMsgIn;
	std_msgs::Float64 tauzMsgIn;
	
	//Callbacks
	void thrustCallBack(const std_msgs::Float64::ConstPtr& msg);
	void tauxCallBack(const std_msgs::Float64::ConstPtr& msg);
	void tauyCallBack(const std_msgs::Float64::ConstPtr& msg);
	void tauzCallBack(const std_msgs::Float64::ConstPtr& msg);

	//Other functions
	void computePosition();
	void computeOrientation();
	void computeRMatrix();
	void sendPose();
	void sendTwist();

};

