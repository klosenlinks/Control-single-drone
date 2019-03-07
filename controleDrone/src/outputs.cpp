#include "ros/ros.h"
#include "outputs.h"

outputprocessor::outputprocessor(const ros::NodeHandle& n): nh(n)
{
	thrustSub = nh.subscribe("/thrust",10,&outputprocessor::thrustCallBack,this);
	tauxSub = nh.subscribe("/taux",10,&outputprocessor::tauxCallBack,this);
	tauySub = nh.subscribe("/tauy",10,&outputprocessor::tauyCallBack,this);
	tauzSub = nh.subscribe("/tauz",10,&outputprocessor::tauzCallBack,this);

	pwd1Pub = nh.advertise<std_msgs::Float64>("",2);
}

void outputprocessor::thrustCallBack(const std_msgs::Float64::ConstPtr& msg){

	thrustMsgIn = *msg;

}

void outputprocessor::tauxCallBack(const std_msgs::Float64::ConstPtr& msg){

	tauxMsgIn = *msg;

}

void outputprocessor::tauyCallBack(const std_msgs::Float64::ConstPtr& msg){

	tauyMsgIn = *msg;

}

void outputprocessor::tauzCallBack(const std_msgs::Float64::ConstPtr& msg){

	tauzMsgIn = *msg;

}

void outputprocessor::processOutputs(){
	pwd1MsgOut.data = sqrt((1/Kpwdt)*((thrustMsgIn.data)/4)+(tauxMsgIn.data/(2*ld))-(tauzMsgIn.data/(4*c))) + 667;
	pwd2MsgOut.data = sqrt((1/Kpwdt)*((thrustMsgIn.data)/4)-(tauxMsgIn.data/(2*ld))-(tauzMsgIn.data/(4*c))) + 667;
	pwd3MsgOut.data = sqrt((1/Kpwdt)*((thrustMsgIn.data)/4)-(tauyMsgIn.data/(2*ld))+(tauzMsgIn.data/(4*c))) + 667;
	pwd4MsgOut.data = sqrt((1/Kpwdt)*((thrustMsgIn.data)/4)+(tauyMsgIn.data/(2*ld))+(tauzMsgIn.data/(4*c))) + 667;

	//Publish
	pwd1Pub.publish(pwd1MsgOut);
	// ...
}


int main(int argc, char**argv)
{
        ros::init(argc, argv, "outputs");
        ros::NodeHandle nh;

	outputprocessor op(nh);	


        ros::Rate Rate(200);
	
	
        while(ros::ok)
        {
		op.processOutputs();
		ros::spinOnce();
                Rate.sleep();
              
        }
}
