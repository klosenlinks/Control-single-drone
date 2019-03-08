#include <ros/ros.h>
#include "stdio.h"
//#include "Eigen/Eigen"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>


class controller{

public:

	controller(const ros::NodeHandle& n); //constructeur
	~controller(){};// destructeur
	void spinController();

private:

	ros::NodeHandle nh;
	//Publishers
	ros::Publisher thrustPub;
	ros::Publisher tauxPub;
	ros::Publisher tauyPub;
	ros::Publisher tauzPub;
	
	//Published variables
	double thrust=0;
	double taux=0;
	double tauy=0;
	double tauz=0;

	//Subscribers
	ros::Subscriber poseSub;
	ros::Subscriber twistSub;
	ros::Subscriber desiredposeSub;
	ros::Subscriber pidgainsSub;

	//measures obtained from subscribers
	tf2::Vector3 axis;
	double xrot=0;
	double yrot=0;
	double zrot=0;
	double w=0;
	tf2::Quaternion quat;
	tf2::Matrix3x3 matrixrot;

	double x=0;
	double y=0;
	double z=0;
	double phi=0;
	double theta=0;
	double psi=0;
	double xd=0;
	double yd=0;
	double zd=0;
	double p = 0;
	double q = 0;
	double r = 0;
	double phid=0;
	double thetad=0;
	double psid=0;
	
	//error values and integrals
	double errz=0;
	double errphi=0;
	double errtheta=0;
	double errpsi=0;

	double interrz=0;
	double interrphi=0;
	double interrpsi=0;
	double interrtheta=0;

	//previous values used for integration
	double errz_=0;
	double errphi_=0;
	double errtheta_=0;
	double errpsi_=0;

	double interrz_=0;
	double interrphi_=0;
	double interrpsi_=0;
	double interrtheta_=0;

	//desired values
	double xdes=0;
	double ydes=0;
	double zdes=0;
	double psides=0;

	double phides=0;
	double thetades=0;

	//PD parameters
	double m=1.066;//cf masse dynamic_model.cpp
	double kp1=0;
	double ki1=0;
	double kd1=0;
	double kp2=0;
	double ki2=0;
	double kd2=0;
	double k1=0;
	double k2=0;
	double kp3=0;
	double ki3=0;
	double kd3=0;
	double k3=0;
	double k4=0;
	double kp4=0;
	double ki4=0;
	double kd4=0;


	//messages
	geometry_msgs::PoseStamped poseMsgIn;
	geometry_msgs::TwistStamped twistMsgIn;
	geometry_msgs::PoseStamped desiredposeMsgIn;
	std_msgs::Float64MultiArray pidgainsMsgIn;

	std_msgs::Float64 thrustMsgOut;
	std_msgs::Float64 tauxMsgOut;
	std_msgs::Float64 tauyMsgOut;
	std_msgs::Float64 tauzMsgOut;

	void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg); //récupère la position et l'orientation du drone
	void twistCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void desiredposeCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void pidgainsCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg);

	void computeThrust();
	void computeTaux();
	void computeTauy();
	void computeTauz();
	void sendToDrone();

};

