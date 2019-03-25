#include "ros/ros.h"
#include "controlsm.h"

#define G -9.81

controller::controller(const ros::NodeHandle& n): nh(n)
{
    //subscriber
    poseSub = nh.subscribe("/pose",2,&controller::poseCallBack,this);
    desiredposeSub = nh.subscribe("/desiredpose",2,&controller::desiredposeCallBack,this);
    twistSub = nh.subscribe("/twist",2,&controller::twistCallBack,this);
    pidgainsSub = nh.subscribe("/pidgains",2,&controller::pidgainsCallBack,this);

    //publisher
    thrustPub = nh.advertise<std_msgs::Float64>("/thrust",2);
    tauxPub = nh.advertise<std_msgs::Float64>("/taux",2);
    tauyPub = nh.advertise<std_msgs::Float64>("/tauy",2);
    tauzPub = nh.advertise<std_msgs::Float64>("/tauz",2);
    errorQuaterPub = nh.advertise<geometry_msgs::Quaternion>("/errorQuater",2);
    quaterdesPub = nh.advertise<geometry_msgs::Quaternion>("/quaterdes",2);
    
}

void controller::pidgainsCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	pidgainsMsgIn = *msg;	

	//kpf=pidgainsMsgIn.data[0];
	//kif=pidgainsMsgIn.data[1];
	//kdf=pidgainsMsgIn.data[2];
}

void controller::desiredposeCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        //on récupère la pose du drone souhaitée
	desiredposeMsgIn = *msg;
	xdes=desiredposeMsgIn.pose.position.x;
	ydes=desiredposeMsgIn.pose.position.y;
	zdes=desiredposeMsgIn.pose.position.z;
}

void controller::poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	
	//on récupère la pose du drone
        poseMsgIn = *msg;
	x=poseMsgIn.pose.position.x;
	y=poseMsgIn.pose.position.y;
	z=poseMsgIn.pose.position.z;
	tf2::convert(poseMsgIn.pose.orientation, orientation_q);
	
}

void controller::twistCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
        //On récupère la vitesse linéaire et angulaire du drone
	twistMsgIn = *msg;
	xd=twistMsgIn.twist.linear.x;
	yd=twistMsgIn.twist.linear.y;
	zd=twistMsgIn.twist.linear.z;
	p=twistMsgIn.twist.angular.x;
	q=twistMsgIn.twist.angular.y;
	r=twistMsgIn.twist.angular.z;

}

void controller::spinController()
{
        //On appelle les différentes fonctions pour envoyer la force et le couple désirés au drone
	computeGlobalForces();
	computeThrust();
	computeQdes();
	computeQerr();
        computeQerrd();
	computeQddes();
	computeOmegades();
	computeSignS();
	computeTorques();
	sendToDrone();
    
}

void controller::computeGlobalForces()
{
	errz_ = errz;
	interrz_ = interrz;

	errz = zdes-z;
        interrz = 0.005*(errz + errz_)*0.5 + interrz_;//intégrale de l'erreur
	
	interrz = (interrz<1*kif)?interrz:1*kif;
        interrz = (interrz>-1*kif)?interrz:-1*kif;

	fx = m*(xdddes + kdf*(xddes-xd) + kpf*(xdes-x));
	fy = m*(ydddes + kdf*(yddes-yd) + kpf*(ydes-y));
	fz = m*(zdddes + kdf*(zddes-zd) + kpf*(zdes-z) + kif*(interrz) - G);

}

void controller::computeThrust()
{
    thrust = sqrt(fx*fx + fy*fy + fz*fz);
    thrust = saturation(thrust,2,20);
    thrustMsgOut.data = thrust;
}

void controller::computeQdes()
{
	dir.setValue(1,0,0); // equivalent a poser psides
        zdrone.setValue(fx/thrust,fy/thrust,fz/thrust);
        ydrone = (zdrone.cross(dir)).normalize();
	xdrone = ydrone.cross(zdrone);
	R.setValue(xdrone.x(),ydrone.x(),zdrone.x(),xdrone.y(),ydrone.y(),zdrone.y(),xdrone.z(),ydrone.z(),zdrone.z());
        /*std::cout<<"xdrone = ("<<xdrone.x()<<";"<<xdrone.y()<<";"<< xdrone.z() << "); "
                 <<"ydrone = ("<<ydrone.x()<<";"<<ydrone.y()<<";"<< ydrone.z() << "); "
                 <<"zdrone = ("<<zdrone.x()<<";"<<zdrone.y()<<";"<< zdrone.z() << ")"<<std::endl;*/
	
	R.getRotation(orientation_qdes);
        orientation_qdes_ = orientation_qdes;

	quaterdes.w = orientation_qdes.w();
	quaterdes.x = orientation_qdes.x();
	quaterdes.y = orientation_qdes.y();
	quaterdes.z = orientation_qdes.z();
	quaterdesPub.publish(quaterdes);
        //std::cout<<"qx="<<orientation_qdes.x()<<"; qy="<<orientation_qdes.y()<<"; qz="<<orientation_qdes.z()<<"; qw="<<orientation_qdes.w()<<std::endl;
}

void controller::computeQerr() 
{
	errorqx_ = errorqx;
	errorqy_ = errorqy;
	errorqz_ = errorqz;

        errorqx = -(-orientation_qdes.x()*orientation_q.w() +orientation_qdes.w()*orientation_q.x() +orientation_qdes.z()*orientation_q.y() -orientation_qdes.y()*orientation_q.z());
        errorqy = -(-orientation_qdes.y()*orientation_q.w() -orientation_qdes.z()*orientation_q.x() +orientation_qdes.w()*orientation_q.y() +orientation_qdes.x()*orientation_q.z());
        errorqz = -(-orientation_qdes.z()*orientation_q.w() +orientation_qdes.y()*orientation_q.x() -orientation_qdes.x()*orientation_q.y() +orientation_qdes.w()*orientation_q.z());

        //debug
        errorQuater.x=errorqx;
        errorQuater.y=errorqy;
        errorQuater.z=errorqz;
        errorQuaterPub.publish(errorQuater);
}

void controller::computeQerrd()
{
	errorqxd = (errorqx-errorqx_)/0.005;
	errorqyd = (errorqy-errorqy_)/0.005;
	errorqzd = (errorqz-errorqz_)/0.005;
	
}

void controller::computeQddes()
{
        qd1des = (orientation_qdes.w() - orientation_qdes_.w())/0.005;
        qd2des = (orientation_qdes.x() - orientation_qdes_.x())/0.005;
        qd3des = (orientation_qdes.y() - orientation_qdes_.y())/0.005;
        qd4des = (orientation_qdes.z() - orientation_qdes_.z())/0.005;

}

void controller::computeOmegades()
{
        pdes = 2.0*(-orientation_qdes.x()*qd1des +orientation_qdes.w()*qd2des +orientation_qdes.z()*qd3des -orientation_qdes.y()*qd4des);
        qdes = 2.0*(-orientation_qdes.y()*qd1des -orientation_qdes.z()*qd2des +orientation_qdes.w()*qd3des +orientation_qdes.x()*qd4des);
        rdes = 2.0*(-orientation_qdes.z()*qd1des +orientation_qdes.y()*qd2des -orientation_qdes.x()*qd3des +orientation_qdes.w()*qd4des);
}

void controller::computeSignS()
{
        sx = (pdes - p) + lambda1*errorqx;
        sy = (qdes - q) + lambda2*errorqy;
        sz = (rdes - r) + lambda3*errorqz;

        signSx = copysign(1,sx); //faire mieux que du bang-bang
        signSy = copysign(1,sy);
        signSz = copysign(1,sz);
}

void controller::computeTorques()
{
	
        taux = 1*Ixx*(1*lambda1*errorqxd + q*r*(Izz-Iyy)/Ixx + k1*signSx);
        tauy = 1*Iyy*(1*lambda2*errorqyd + p*r*(Ixx-Izz)/Iyy + k2*signSy);
        tauz = 1*Izz*(1*lambda3*errorqzd + p*q*(Iyy-Ixx)/Izz + k3*signSz);

        taux=saturation(taux,-0.5,0.5);
        tauy=saturation(tauy,-0.5,0.5);
        tauz=saturation(tauz,-0.1,0.1);

	tauxMsgOut.data = taux;
	tauyMsgOut.data = tauy;
	tauzMsgOut.data = tauz;
}

void controller::sendToDrone()
{
// on publie la commande en vitesse angulaire au drone calculée à l'aide de la poussée calculée
	thrustPub.publish(thrustMsgOut);
	tauxPub.publish(tauxMsgOut);
	tauyPub.publish(tauyMsgOut);
	tauzPub.publish(tauzMsgOut);
}

double saturation(double x,double min,double max)
{
    double res = x;
    if(x>max){
        res = max;
    }
    if(x<min){
        res = min;
    }
    return res;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "controlsm");
    ros::NodeHandle nh;

    controller drone(nh);
    ros::Rate Rate(200);

    /*int i=0;
    while(i<(200*3)) //délai avant décollage
    {
        drone.thrustMsgOut.data=3;
        drone.tauxMsgOut.data=0;
        drone.tauyMsgOut.data=0;
        drone.tauzMsgOut.data=0;
        drone.sendToDrone();
        Rate.sleep();
        ros::spinOnce();
        i++;
    }*/

    while(ros::ok)
    {
        drone.spinController();
        Rate.sleep();
        ros::spinOnce();
    }
}
