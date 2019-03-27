//ROS
#include <ros/ros.h>

//Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

//Math
#include <cmath>

//Gravity constant
#define G -9.81

class dynamicModel{

public:

	dynamicModel(const ros::NodeHandle& n,float m, float ixx, float iyy, float izz); 
	~dynamicModel(){};
	void spinModel();

private:
	
	ros::NodeHandle nh;
	
	//Drone parameters
	float mass;
	float ixx;
	float iyy;
	float izz;

	//Inputs
	float thrust=0;
	float taux=0;
	float tauy=0;
	float tauz=0;
	
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

	float p=0;
	float pd=0;

	float q=0;
	float qd=0;

	float r=0;
	float rd=0;
	
	//Previous values stored for integration	
	float x_=0;
	float xd_=0;
	float xdd_=0;

	float y_=0;
	float yd_=0;
	float ydd_=0;

	float z_=0;
	float zd_=0;
	float zdd_=0;

	float p_=0;
	float pd_=0;

	float q_=0;
	float qd_=0;

	float r_=0;
	float rd_=0;
	
        //Quaternion
        float q0=1.0;
        float q1=0;
        float q2=0;
        float q3=0;

        float q0_=1.0;
        float q1_=0;
        float q2_=0;
        float q3_=0;
	
	//Quaternion derivatives
        float q0d=0;
        float q1d=0;
        float q2d=0;
        float q3d=0;

        float q0d_=0;
        float q1d_=0;
        float q2d_=0;
        float q3d_=0;

	//Rotation matrix
	//We only need the last column (ax,ay,az) here
	float ax=0;
	float ay=0;
	float az=0;

	//Publishers 
	ros::Publisher posePub;
	ros::Publisher twistPub;

	//Subscribers
	ros::Subscriber thrustSub;
	ros::Subscriber tauxSub;
	ros::Subscriber tauySub;
	ros::Subscriber tauzSub;	

	//Messages
	geometry_msgs::PoseStamped poseMsgOut;
	geometry_msgs::TwistStamped twistMsgOut;
	
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
