#include "outputs.h"


outputprocessor::outputprocessor(const ros::NodeHandle& n): nh(n)
{
    //Subscribers
    thrustSub = nh.subscribe("/thrust",10,&outputprocessor::thrustCallBack,this);
    tauxSub = nh.subscribe("/taux",10,&outputprocessor::tauxCallBack,this);
    tauySub = nh.subscribe("/tauy",10,&outputprocessor::tauyCallBack,this);
    tauzSub = nh.subscribe("/tauz",10,&outputprocessor::tauzCallBack,this);

    //Publishers
    pwdPub = nh.advertise<mavros_msgs::ActuatorControl>("/drone1/mavros/actuator_control",2);
}

void outputprocessor::thrustCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    thrustMsgIn = *msg;
}

void outputprocessor::tauxCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    tauxMsgIn = *msg;
}

void outputprocessor::tauyCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    tauyMsgIn = *msg;
}

void outputprocessor::tauzCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    tauzMsgIn = *msg;
}

void outputprocessor::processOutputs()
{
    //We apply our forces and moments to the 4 motors using the distribution matrix
    pwdMsgOut.controls[0] = sqrt(max0x((1.0/Kpwdt)*((thrustMsgIn.data/4.0)-(tauxMsgIn.data/(2.0*ld))-(tauzMsgIn.data/(4.0*c))))) + 667.0;
    pwdMsgOut.controls[1] = sqrt(max0x((1.0/Kpwdt)*((thrustMsgIn.data/4.0)+(tauxMsgIn.data/(2.0*ld))-(tauzMsgIn.data/(4.0*c))))) + 667.0;
    pwdMsgOut.controls[2] = sqrt(max0x((1.0/Kpwdt)*((thrustMsgIn.data/4.0)-(tauyMsgIn.data/(2.0*ld))+(tauzMsgIn.data/(4.0*c))))) + 667.0;
    pwdMsgOut.controls[3] = sqrt(max0x((1.0/Kpwdt)*((thrustMsgIn.data/4.0)+(tauyMsgIn.data/(2.0*ld))+(tauzMsgIn.data/(4.0*c))))) + 667.0;

    for (int i=0;i<4;i++)
    {
        pwdMsgOut.controls[i] = (2*pwdMsgOut.controls[i] - (1075 + 1950))/(1950-1075);
    }

    // We born between -1 et 1
    pwdMsgOut.controls[0] = born(pwdMsgOut.controls[0]);
    pwdMsgOut.controls[1] = born(pwdMsgOut.controls[1]);
    pwdMsgOut.controls[2] = born(pwdMsgOut.controls[2]);
    pwdMsgOut.controls[3] = born(pwdMsgOut.controls[3]);

    //Publish
    pwdPub.publish(pwdMsgOut);
}

float outputprocessor::max0x(float x)
{
    //We want positive values under sqrt()
    return (x<0)?0:x;
}

double born(double num)
{
    //Born between -1 and 1
    double res = num;

    if(num < -1){
            res = -1;
    }
    if(num > 1){
            res = 1;
    }
    return res;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "outputs");
    ros::NodeHandle nh;
    ros::Rate Rate(200);

    outputprocessor op(nh);

    while(ros::ok)
    {
        op.processOutputs();
        ros::spinOnce();
        Rate.sleep();
    }
}
