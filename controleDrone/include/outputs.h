#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>

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
	ros::Publisher pwd1Pub;

	//Messages
	std_msgs::Float64 pwd1MsgOut;
	std_msgs::Float64 pwd2MsgOut;
	std_msgs::Float64 pwd3MsgOut;
	std_msgs::Float64 pwd4MsgOut;

	std_msgs::Float64 thrustMsgIn;
	std_msgs::Float64 tauxMsgIn;
	std_msgs::Float64 tauyMsgIn;
	std_msgs::Float64 tauzMsgIn;

	//Constantes
	double Kpwdt = 3.255*pow(10,-6);
	double Kpwdd = 8*pow(10,-9);
	double c = Kpwdd/Kpwdt;
	double ld = 0.2; // A mesurer


void thrustCallBack(const std_msgs::Float64::ConstPtr& msg);
void tauxCallBack(const std_msgs::Float64::ConstPtr& msg);
void tauyCallBack(const std_msgs::Float64::ConstPtr& msg);
void tauzCallBack(const std_msgs::Float64::ConstPtr& msg);


};
