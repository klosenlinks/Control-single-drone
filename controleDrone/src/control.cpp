#include "ros/ros.h"
#include "control.h"

#define G -9.81

controller::controller(const ros::NodeHandle& n): nh(n)
{
    //Definir l'objectif
    //xdes=20;
    //ydes=40;
    //zdes=60;
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

}

void controller::pidgainsCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	pidgainsMsgIn = *msg;	

	kp1=pidgainsMsgIn.data[0];
	ki1=pidgainsMsgIn.data[1];
	kd1=pidgainsMsgIn.data[2];
	kp2=pidgainsMsgIn.data[3];
	ki2=pidgainsMsgIn.data[4];
	kd2=pidgainsMsgIn.data[5];
	k1=pidgainsMsgIn.data[6];
	k2=pidgainsMsgIn.data[7];
	kp3=pidgainsMsgIn.data[8];
	ki3=pidgainsMsgIn.data[9];
	kd3=pidgainsMsgIn.data[10];
	k3=pidgainsMsgIn.data[11];
	k4=pidgainsMsgIn.data[12];
	kp4=pidgainsMsgIn.data[13];
	ki4=pidgainsMsgIn.data[14];
	kd4=pidgainsMsgIn.data[15];

	// [5,0,7,4,0,5,0.05,0.5,4,0,5,0.05,0.5,2,0,4]
}

void controller::desiredposeCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	desiredposeMsgIn = *msg;
	xdes=desiredposeMsgIn.pose.position.x;
	ydes=desiredposeMsgIn.pose.position.y;
	zdes=desiredposeMsgIn.pose.position.z;
	psides=0;
}

void controller::poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	
	//on récupère la pose du drone
	poseMsgIn = *msg; //copie de msg dans poseMsgIn
	x=poseMsgIn.pose.position.x;
	y=poseMsgIn.pose.position.y;
	z=poseMsgIn.pose.position.z;
	tf2::convert(poseMsgIn.pose.orientation, quat);
	matrixrot.setRotation(quat);
	
	matrixrot.getRPY(phi,theta,psi,1);

	
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

	phid = cos(theta)*p + sin(theta)*r;
	thetad = sin(theta)*tan(phi)*p + q - cos(theta)*tan(phi)*r;
	psid = -1*(sin(theta)/cos(phi))*p + (cos(theta)/cos(phi))*r;
}

void controller::spinController()
{
    computeThrust();
    computeTaux();
    computeTauy();
    computeTauz();
    sendToDrone();
}

void controller::computeThrust()
{
    errz_ = errz;
    interrz_ = interrz;

    errz = zdes-z;
    interrz = 0.005*(errz + errz_)/2 + interrz_;

    thrust = -1*m*G + kp1*(zdes-z) + kd1*-1*zd + ki1*interrz;
    thrustMsgOut.data = (thrust < 0)?0:thrust; // si la commande devait etre négative, on eteint les moteurs

}

void controller::computeTaux()
{
	phides = -1*atan2(k1*(ydes-y)-k2*yd,1);

	errphi_ = errphi;
	interrphi_ = interrphi;

	errphi = phides-phi;
	interrphi = 0.005*(errphi + errphi_)/2 + interrphi_;
	
	taux = kp2*(phides-phi) + kd2*-1*phid + ki2*interrphi;
	//taux = -1*(kp2*(ydes-y) + kd2*(-1)*yd);
	tauxMsgOut.data = taux;
}

void controller::computeTauy()
{
	thetades = atan2(k3*(xdes-x)-k4*xd,1);

	errtheta_ = errtheta;
	interrtheta_ = interrtheta;

	errtheta = thetades-theta;
	interrtheta = 0.005*(errtheta + errtheta_)/2 + interrtheta_;

	tauy = kp3*(thetades-theta) + kd3*-1*thetad +ki3*interrtheta;
	//tauy = -1*(kp3*(xdes-x) + kd3*(-1)*xd);
	tauyMsgOut.data = tauy;
}

void controller::computeTauz()
{	
	errpsi_ = errpsi;
	interrpsi_ = interrpsi;

	errpsi = psides-psi;
	interrpsi = 0.005*(errpsi + errpsi_)/2 + interrpsi_;

	tauz = kp4*(psides-psi) + kd4*-1*psid + ki4*interrpsi;
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

