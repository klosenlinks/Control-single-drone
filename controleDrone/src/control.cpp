#include "ros/ros.h"
#include "control.h"

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
}

void controller::pidgainsCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	pidgainsMsgIn = *msg;	

	kpf=pidgainsMsgIn.data[0];
	kif=pidgainsMsgIn.data[1];
	kdf=pidgainsMsgIn.data[2];
	kptauxy=pidgainsMsgIn.data[3];
	kitauxy=pidgainsMsgIn.data[4]; //inutilisé
	kdtauxy=pidgainsMsgIn.data[5];
	kptauz=pidgainsMsgIn.data[6];
	kitauz=pidgainsMsgIn.data[7]; //inutilisé
	kdtauz=pidgainsMsgIn.data[8];
}

void controller::desiredposeCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	desiredposeMsgIn = *msg;
	xdes=desiredposeMsgIn.pose.position.x;
	ydes=desiredposeMsgIn.pose.position.y;
	zdes=desiredposeMsgIn.pose.position.z;
}

void controller::poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	
	//on récupère la pose du drone
	poseMsgIn = *msg; //copie de msg dans poseMsgIn
	x=poseMsgIn.pose.position.x;
	y=poseMsgIn.pose.position.y;
	z=poseMsgIn.pose.position.z;

	orientation_q_ = orientation_q;
	tf2::convert(poseMsgIn.pose.orientation, orientation_q);
	
}

void controller::twistCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
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
	computeGlobalForces();
	computeThrust();
	computeQdes();
	computeQerr();
	computeQddes();
	computeOmegades();
	computeTorques();
	sendToDrone();
    
}

void controller::computeGlobalForces()
{
	errz_ = errz;
	interrz_ = interrz;

	errz = zdes-z;
	interrz = 0.005*(errz + errz_)/2 + interrz_;
	
	interrz = (interrz<10*kif)?interrz:10*kif;


	fx = m*(xdddes + kdf*(xddes-xd) + kpf*(xdes-x));
	fy = m*(ydddes + kdf*(yddes-yd) + kpf*(ydes-y));
	fz = m*(zdddes + kdf*(zddes-zd) + kpf*(zdes-z) + kif*(interrz) - G);
}

void controller::computeThrust()
{
    thrust = sqrt(fx*fx + fy*fy + fz*fz);
    thrustMsgOut.data = thrust;
//    std::cout<<"fx="<<fx<<"; fy="<<fy<<"; fz="<<fz<<std::endl;
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
        //std::cout<<"qx="<<orientation_qdes.x()<<"; qy="<<orientation_qdes.y()<<"; qz="<<orientation_qdes.z()<<"; qw="<<orientation_qdes.w()<<std::endl;
}

void controller::computeQerr() 
{
        errorqx = -(-orientation_qdes.x()*orientation_q.w() +orientation_qdes.w()*orientation_q.x() +orientation_qdes.z()*orientation_q.y() -orientation_qdes.y()*orientation_q.z());
        errorqy = -(-orientation_qdes.y()*orientation_q.w() -orientation_qdes.z()*orientation_q.x() +orientation_qdes.w()*orientation_q.y() +orientation_qdes.x()*orientation_q.z());
        errorqz = -(-orientation_qdes.z()*orientation_q.w() +orientation_qdes.y()*orientation_q.x() -orientation_qdes.x()*orientation_q.y() +orientation_qdes.w()*orientation_q.z());

        //debug
        errorQuater.x=errorqx;
        errorQuater.y=errorqy;
        errorQuater.z=errorqz;
        errorQuaterPub.publish(errorQuater);
}

void controller::computeQddes()
{
        qd1 = (orientation_q.w() - orientation_q_.w())/0.005;
        qd2 = (orientation_q.x() - orientation_q_.x())/0.005;
        qd3 = (orientation_q.y() - orientation_q_.y())/0.005;
        qd4 = (orientation_q.z() - orientation_q_.z())/0.005;
}

void controller::computeOmegades()
{
        pdes = 2*(-orientation_qdes.x()*qd1 +orientation_qdes.w()*qd2 +orientation_qdes.z()*qd3 -orientation_qdes.y()*qd4);
        qdes = 2*(-orientation_qdes.y()*qd1 -orientation_qdes.z()*qd2 +orientation_qdes.w()*qd3 +orientation_qdes.x()*qd4);
        rdes = 2*(-orientation_qdes.z()*qd1 +orientation_qdes.y()*qd2 -orientation_qdes.x()*qd3 +orientation_qdes.w()*qd4);
}

void controller::computeTorques()
{
	taux = kdtauxy*(pdes-p) + kptauxy*errorqx;
	tauy = kdtauxy*(qdes-q) + kptauxy*errorqy;
        tauz = kdtauz *(rdes-r) + kptauz *errorqz;

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

int main(int argc, char**argv)
{
        ros::init(argc, argv, "control");
        ros::NodeHandle nh;

        controller drone(nh);
        ros::Rate Rate(200);
	
	

        while(ros::ok)
        {
                drone.spinController();
                Rate.sleep();
                ros::spinOnce();
        }
}

