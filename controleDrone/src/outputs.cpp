#include "ros/ros.h"
#include "outputs.h"


outputprocessor::outputprocessor(const ros::NodeHandle& n): nh(n)
{
	thrustSub = nh.subscribe("/thrust",10,&outputprocessor::thrustCallBack,this);
	tauxSub = nh.subscribe("/taux",10,&outputprocessor::tauxCallBack,this);
	tauySub = nh.subscribe("/tauy",10,&outputprocessor::tauyCallBack,this);
	tauzSub = nh.subscribe("/tauz",10,&outputprocessor::tauzCallBack,this);

	pwdPub = nh.advertise<mavros_msgs::ActuatorControl>("/drone1/mavros/actuator_control",2);
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

		
	

	pwdMsgOut.controls[0] = sqrt(max0x((1/Kpwdt)*((thrustMsgIn.data)/4)+(tauxMsgIn.data/(2*ld))-(tauzMsgIn.data/(4*c)))) + 667;
	pwdMsgOut.controls[1] = sqrt(max0x((1/Kpwdt)*((thrustMsgIn.data)/4)-(tauxMsgIn.data/(2*ld))-(tauzMsgIn.data/(4*c)))) + 667;
	pwdMsgOut.controls[2] = sqrt(max0x((1/Kpwdt)*((thrustMsgIn.data)/4)-(tauyMsgIn.data/(2*ld))+(tauzMsgIn.data/(4*c)))) + 667;
	pwdMsgOut.controls[3] = sqrt(max0x((1/Kpwdt)*((thrustMsgIn.data)/4)+(tauyMsgIn.data/(2*ld))+(tauzMsgIn.data/(4*c)))) + 667;
	
	

        for (int i=0;i<4;i++){
            pwdMsgOut.controls[i] = (2*pwdMsgOut.controls[i] - (1075 + 1950))/(1950-1075);
        }
	
	// On borne entre -1 et 1
	pwdMsgOut.controls[0] = (((pwdMsgOut.controls[0]<-1)?-1:pwdMsgOut.controls[0])>1)?1:pwdMsgOut.controls[0]; 
	pwdMsgOut.controls[1] = (((pwdMsgOut.controls[1]<-1)?-1:pwdMsgOut.controls[1])>1)?1:pwdMsgOut.controls[1];
	pwdMsgOut.controls[2] = (((pwdMsgOut.controls[2]<-1)?-1:pwdMsgOut.controls[2])>1)?1:pwdMsgOut.controls[2];
	pwdMsgOut.controls[3] = (((pwdMsgOut.controls[3]<-1)?-1:pwdMsgOut.controls[3])>1)?1:pwdMsgOut.controls[3];

	//Publish
	pwdPub.publish(pwdMsgOut);
	
}

float outputprocessor::max0x(float x){

	return (x<0)?0:x;

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
