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

		
	

        pwdMsgOut.controls[0] = sqrt(max0x((1.0/Kpwdt)*((thrustMsgIn.data/4.0)-(tauxMsgIn.data/(2.0*ld))-(tauzMsgIn.data/(4.0*c))))) + 667.0;
        pwdMsgOut.controls[1] = sqrt(max0x((1.0/Kpwdt)*((thrustMsgIn.data/4.0)+(tauxMsgIn.data/(2.0*ld))-(tauzMsgIn.data/(4.0*c))))) + 667.0;
        pwdMsgOut.controls[2] = sqrt(max0x((1.0/Kpwdt)*((thrustMsgIn.data/4.0)-(tauyMsgIn.data/(2.0*ld))+(tauzMsgIn.data/(4.0*c))))) + 667.0;
        pwdMsgOut.controls[3] = sqrt(max0x((1.0/Kpwdt)*((thrustMsgIn.data/4.0)+(tauyMsgIn.data/(2.0*ld))+(tauzMsgIn.data/(4.0*c))))) + 667.0;

	
	

        for (int i=0;i<4;i++){
            pwdMsgOut.controls[i] = (2*pwdMsgOut.controls[i] - (1075 + 1950))/(1950-1075);
        }
	
	// On borne entre -1 et 1
	pwdMsgOut.controls[0] = born(pwdMsgOut.controls[0]); 
	pwdMsgOut.controls[1] = born(pwdMsgOut.controls[1]);
	pwdMsgOut.controls[2] = born(pwdMsgOut.controls[2]);
	pwdMsgOut.controls[3] = born(pwdMsgOut.controls[3]);
	
	//Publish
	pwdPub.publish(pwdMsgOut);

	/*
	double foo1;
	double foo2;
	double foo3;
	double foo4;

	foo1 = 0.5*(pwdMsgOut.controls[0]*(1950-1075)+1075+1950);
	foo2 = 0.5*(pwdMsgOut.controls[1]*(1950-1075)+1075+1950);
	foo3 = 0.5*(pwdMsgOut.controls[2]*(1950-1075)+1075+1950);
	foo4 = 0.5*(pwdMsgOut.controls[3]*(1950-1075)+1075+1950);

	foo1 = pow(foo1-667,2)*Kpwdt;
	foo2 = pow(foo2-667,2)*Kpwdt;
	foo3 = pow(foo3-667,2)*Kpwdt;
	foo4 = pow(foo4-667,2)*Kpwdt;
        
	double foothrust;
	double footaux;
	double footauy;
	double footauz;

	foothrust = foo1+foo2+foo3+foo4;
	footaux = ld*(foo1-foo2);
	footauy = ld*(foo4-foo3);
	footauz = c*(foo3+foo4-foo1-foo2);

	std::cout<<"calc thrust = "<<foothrust<<std::endl;
	std::cout<<"calc taux = "<<footaux<<std::endl;
	std::cout<<"calc tauy = "<<footauy<<std::endl;
	std::cout<<"calc tauz = "<<footauz<<std::endl;
	*/
	
}

double born(double num) {
	double res = num;
	if(num < -1){
		res = -1;	
	}	
	if(num > 1){
		res = 1;	
	}
	return res;
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
