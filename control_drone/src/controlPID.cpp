#include "controlPID.h"


controller::controller(const ros::NodeHandle& n): nh(n)
{
    //Subscribers
    poseSub = nh.subscribe("/pose",2,&controller::poseCallBack,this);
    desiredposeSub = nh.subscribe("/desiredpose",2,&controller::desiredposeCallBack,this);
    twistSub = nh.subscribe("/twist",2,&controller::twistCallBack,this);

    //Publishers
    thrustPub = nh.advertise<std_msgs::Float64>("/thrust",2);
    tauxPub = nh.advertise<std_msgs::Float64>("/taux",2);
    tauyPub = nh.advertise<std_msgs::Float64>("/tauy",2);
    tauzPub = nh.advertise<std_msgs::Float64>("/tauz",2);

    //Initialization of the wrench before take off
    thrustMsgOut.data=3;
    tauxMsgOut.data=0;
    tauyMsgOut.data=0;
    tauzMsgOut.data=0;
}

void controller::desiredposeCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //We recover the desired pose of the drone
    desiredposeMsgIn = *msg;
    xdes=desiredposeMsgIn.pose.position.x;
    ydes=desiredposeMsgIn.pose.position.y;
    zdes=desiredposeMsgIn.pose.position.z;
}

void controller::poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //We recover the actual drone pose
    poseMsgIn = *msg;
    x=poseMsgIn.pose.position.x;
    y=poseMsgIn.pose.position.y;
    z=poseMsgIn.pose.position.z;
    tf2::convert(poseMsgIn.pose.orientation, orientation_q);
}

void controller::twistCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    //We recover the angular and linear speed of the drone
    twistMsgIn = *msg;
    xd=twistMsgIn.twist.linear.x;
    yd=twistMsgIn.twist.linear.y;
    zd=twistMsgIn.twist.linear.z;
    p=twistMsgIn.twist.angular.x;
    q=twistMsgIn.twist.angular.y;
    r=twistMsgIn.twist.angular.z;
}

void controller::computeGlobalForces()
{
    //We save our previous error values
    errx_ = errx;
    interrx_ = interrx;
    erry_ = erry;
    interry_ = interry;
    errz_ = errz;
    interrz_ = interrz;

    //We compute the error on
    errx = xdes-x;
    erry = ydes-y;
    errz = zdes-z;

    //We integrate the error
    interrx = 0.005*(errx + errx_)*0.5 + interrx_;
    interry = 0.005*(erry + erry_)*0.5 + interry_;
    interrz = 0.005*(errz + errz_)*0.5 + interrz_;

    //We limit the integration so as not to diverge the command (anti-windup)
    interrx = (interrx<5*kif)?interrx:5*kif;
    interrx = (interrx>-5*kif)?interrx:-5*kif;
    interry = (interry<5*kif)?interry:5*kif;
    interry = (interry>-5*kif)?interry:-5*kif;
    interrz = (interrz<5*kif)?interrz:5*kif;
    interrz = (interrz>-5*kif)?interrz:-5*kif;

    //We compute the forces with a PID command
    fx = m*(xdddes + kdf*(xddes-xd) + kpf*(xdes-x) + kif*(interrx));
    fy = m*(ydddes + kdf*(yddes-yd) + kpf*(ydes-y) + kif*(interry));
    fz = m*(zdddes + kdf*(zddes-zd) + kpf*(zdes-z) + kif*(interrz) - G);
}

void controller::computeThrust()
{
    //We compute the necessary force to reach the desired position
    thrust = sqrt(fx*fx + fy*fy + fz*fz);
    thrust = saturation(thrust,2,20);
    thrustMsgOut.data = thrust;
}

void controller::computeQdes()
{
    //the direction of the thrust gives us the desired reference frame of the drone
    //the x-axis of the drone is arbitrary chosen to match the global x-axis
    dir.setValue(1,0,0);
    zdrone.setValue(fx/thrust,fy/thrust,fz/thrust);
    ydrone = (zdrone.cross(dir)).normalize();
    xdrone = ydrone.cross(zdrone);

    //We compute the rotation matrix
    R.setValue(xdrone.x(),ydrone.x(),zdrone.x(),xdrone.y(),ydrone.y(),zdrone.y(),xdrone.z(),ydrone.z(),zdrone.z());

    //We convert the rotation matrix to a quaternion
    R.getRotation(orientation_qdes);
    orientation_qdes_ = orientation_qdes;
}

void controller::computeQerr() 
{
    //Quaternion error
    errorqx = -(-orientation_qdes.x()*orientation_q.w() +orientation_qdes.w()*orientation_q.x() +orientation_qdes.z()*orientation_q.y() -orientation_qdes.y()*orientation_q.z());
    errorqy = -(-orientation_qdes.y()*orientation_q.w() -orientation_qdes.z()*orientation_q.x() +orientation_qdes.w()*orientation_q.y() +orientation_qdes.x()*orientation_q.z());
    errorqz = -(-orientation_qdes.z()*orientation_q.w() +orientation_qdes.y()*orientation_q.x() -orientation_qdes.x()*orientation_q.y() +orientation_qdes.w()*orientation_q.z());
}

void controller::computeQddes()
{
    //We derive the desired quaternion
    qd1des = (orientation_qdes.w() - orientation_qdes_.w())/0.005;
    qd2des = (orientation_qdes.x() - orientation_qdes_.x())/0.005;
    qd3des = (orientation_qdes.y() - orientation_qdes_.y())/0.005;
    qd4des = (orientation_qdes.z() - orientation_qdes_.z())/0.005;
}

void controller::computeOmegades()
{
    //We compute the desired angular speed
    pdes = 2.0*(-orientation_qdes.x()*qd1des +orientation_qdes.w()*qd2des +orientation_qdes.z()*qd3des -orientation_qdes.y()*qd4des);
    qdes = 2.0*(-orientation_qdes.y()*qd1des -orientation_qdes.z()*qd2des +orientation_qdes.w()*qd3des +orientation_qdes.x()*qd4des);
    rdes = 2.0*(-orientation_qdes.z()*qd1des +orientation_qdes.y()*qd2des -orientation_qdes.x()*qd3des +orientation_qdes.w()*qd4des);
}

void controller::computeTorques()
{
    //We compute the moments with a PD command
    taux = kdtauxy*(pdes-p) + kptauxy*errorqx;
    tauy = kdtauxy*(qdes-q) + kptauxy*errorqy;
    tauz = kdtauz *(rdes-r) + kptauz *errorqz;

    //We saturate the moments so the drone doesn't turn around
    taux=saturation(taux,-0.5,0.5);
    tauy=saturation(tauy,-0.5,0.5);
    tauz=saturation(tauz,-0.1,0.1);

    tauxMsgOut.data = taux;
    tauyMsgOut.data = tauy;
    tauzMsgOut.data = tauz;
}

void controller::sendToDrone()
{
    //We publish the calculated thrust and moments
    thrustPub.publish(thrustMsgOut);
    tauxPub.publish(tauxMsgOut);
    tauyPub.publish(tauyMsgOut);
    tauzPub.publish(tauzMsgOut);
}

void controller::spinController()
{
    //The different functions are called to send the desired force and torque to the drone
    computeGlobalForces();
    computeThrust();
    computeQdes();
    computeQerr();
    computeQddes();
    computeOmegades();
    computeTorques();
    sendToDrone();
}

double saturation(double x,double min,double max)
{
    double res = x;

    if(x>max)
    {
        res = max;
    }

    if(x<min)
    {
        res = min;
    }

    return res;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "controlPID");
    ros::NodeHandle nh;
    ros::Rate Rate(200); //200Hz

    controller drone(nh);

    int i=0;
    while(i<(200*3)) //delay before take off
    {
        drone.sendToDrone(); //We don't send enough thrust for the drone to take off
        Rate.sleep();
        ros::spinOnce();
        i++;
    }

    while(ros::ok)
    {
        drone.spinController();
        Rate.sleep();
        ros::spinOnce();
    }
}
