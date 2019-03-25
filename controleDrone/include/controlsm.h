#include <ros/ros.h>
#include "stdio.h"
//#include "Eigen/Eigen"
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Quaternion.h>//

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>


class controller{

public:

	controller(const ros::NodeHandle& n); //constructeur
	~controller(){};// destructeur
	void spinController();
        void sendToDrone();

        std_msgs::Float64 thrustMsgOut;
        std_msgs::Float64 tauxMsgOut;
        std_msgs::Float64 tauyMsgOut;
        std_msgs::Float64 tauzMsgOut;

private:

	ros::NodeHandle nh;
	//Publishers
	ros::Publisher thrustPub;
	ros::Publisher tauxPub;
	ros::Publisher tauyPub;
	ros::Publisher tauzPub;
	ros::Publisher errorQuaterPub;
	ros::Publisher quaterdesPub;
	
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
        tf2::Quaternion orientation_q  = tf2::Quaternion(0,0,0,1);
        tf2::Quaternion orientation_qdes_ = tf2::Quaternion(0,0,0,1);

	double x=0;
	double y=0;
	double z=0;

	double xd=0;
	double yd=0;
	double zd=0;

	double p = 0;
	double q = 0;
	double r = 0;
	
	//error values and integrals
	double errz=0;
	double interrz=0;

	//previous values used for integration
	double errz_=0;
	double interrz_=0;

	//desired values
	double xdes=0;
	double ydes=0;
	double zdes=0;

	double xddes=0;
	double yddes=0;
	double zddes=0;

	double xdddes=0;
	double ydddes=0;
	double zdddes=0;

	tf2::Quaternion orientation_qdes;
	
	//global forces
	double fx=0;
	double fy=0;
	double fz=0;

	//Orientation computation
	tf2::Vector3 xdrone;
	tf2::Vector3 ydrone;
	tf2::Vector3 zdrone;
	tf2::Vector3 dir;
	
	tf2::Matrix3x3 R;
	double errorqx=0;
	double errorqy=0;
	double errorqz=0;

	double errorqx_=0;
	double errorqy_=0;
	double errorqz_=0;

	double errorqxd;
	double errorqyd;
	double errorqzd;

        double qd1des;
        double qd2des;
        double qd3des;
        double qd4des;

	double pdes;
	double qdes;
	double rdes;

	//sliding mode
	double m=1.066;//cf masse dynamic_model.cpp
	double kpf=4; //4
	double kif=1; //1
	double kdf=4; //4
	
	double sx=0;
	double sy=0;
	double sz=0;
	double signSx=0;
	double signSy=0;
	double signSz=0;
	double Ixx=0.1152;
	double Iyy=0.1152;
	double Izz=0.218;
	double lambda1=4;
	double lambda2=4;
	double lambda3=4;
	double k1=1;
	double k2=1;
	double k3=1;


	//messages
	geometry_msgs::PoseStamped poseMsgIn;
	geometry_msgs::TwistStamped twistMsgIn;
	geometry_msgs::PoseStamped desiredposeMsgIn;
	std_msgs::Float64MultiArray pidgainsMsgIn;

	geometry_msgs::Quaternion errorQuater;	
	geometry_msgs::Quaternion quaterdes;
	


	void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg); //récupère la position et l'orientation du drone
	void twistCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void desiredposeCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void pidgainsCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg);

	void computeThrust();
	void computeGlobalForces();
	void computeQdes();
	void computeQerr();
	void computeQerrd();
	void computeQddes();
	void computeOmegades();
	void computeSignS();
	void computeTorques();

	

};

double saturation(double x, double min, double max);
