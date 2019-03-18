#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <mavros_msgs/ActuatorControl.h>

class outputprocessor{

public:

	outputprocessor(const ros::NodeHandle& n); //constructeur
	~outputprocessor(){};// destructeur
	void processOutputs();

private:

	ros::NodeHandle nh;
	//Subscribers
	ros::Subscriber thrustSub;
	ros::Subscriber tauxSub;
	ros::Subscriber tauySub;
	ros::Subscriber tauzSub;

	//Publishers
	ros::Publisher pwdPub;

	//Messages
	mavros_msgs::ActuatorControl pwdMsgOut;

	std_msgs::Float64 thrustMsgIn;
	std_msgs::Float64 tauxMsgIn;
	std_msgs::Float64 tauyMsgIn;
	std_msgs::Float64 tauzMsgIn;

	//Constantes
	double Kpwdt = 3.255*pow(10,-6);
	double Kpwdd = 8*pow(10,-9);
	double c = Kpwdd/Kpwdt;
	double ld = 0.17; // A mesurer


void thrustCallBack(const std_msgs::Float64::ConstPtr& msg);
void tauxCallBack(const std_msgs::Float64::ConstPtr& msg);
void tauyCallBack(const std_msgs::Float64::ConstPtr& msg);
void tauzCallBack(const std_msgs::Float64::ConstPtr& msg);
float max0x(float x);


};

double born(double num);
