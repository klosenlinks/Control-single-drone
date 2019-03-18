#include "ros/ros.h"
#include <dynamic_model.h>

#define G -9.81

//Construtor
dynamicModel::dynamicModel(const ros::NodeHandle& n, float m, float ixx, float iyy, float izz): nh(n), mass(m), ixx(ixx), iyy(iyy), izz(izz)
{

	//subscriber
	thrustSub = nh.subscribe("/thrust",2,&dynamicModel::thrustCallBack,this);
	tauxSub = nh.subscribe("/taux",2,&dynamicModel::tauxCallBack,this);
	tauySub = nh.subscribe("/tauy",2,&dynamicModel::tauyCallBack,this);
	tauzSub = nh.subscribe("/tauz",2,&dynamicModel::tauzCallBack,this);
	//publisher
	posePub = nh.advertise<geometry_msgs::PoseStamped>("/pose",2);
	twistPub = nh.advertise<geometry_msgs::TwistStamped>("/twist",2);

}

//Callbacks
void dynamicModel::thrustCallBack(const std_msgs::Float64::ConstPtr& msg)
{
	thrust = msg->data;
}

void dynamicModel::tauxCallBack(const std_msgs::Float64::ConstPtr& msg)
{
	taux = msg->data;
}

void dynamicModel::tauyCallBack(const std_msgs::Float64::ConstPtr& msg)
{
	tauy = msg->data;
}

void dynamicModel::tauzCallBack(const std_msgs::Float64::ConstPtr& msg)
{
	tauz = msg->data;
}


void dynamicModel::computePosition()
{
	//on calcule la position du drone à partir des valeurs précédentes et
	//des forces appliquées au drone. On utilise la méthode des trapèzes
	//pour intégrer
	

	//First we store the previous values to use in integration
	xdd_ = xdd;
	xd_ = xd;
	x_ = x;

	ydd_ = ydd;
	yd_ = yd;
	y_ = y;

	zdd_ = zdd;
	zd_ = zd;
	z_ = z;

	//Then we calculate the pose and its derivatives
	xdd = ax*thrust/mass;
        xd = 0.005*(xdd+xdd_)*0.5 + xd_;
        x = 0.005*(xd+xd_)*0.5 + x_;

	ydd = ay*thrust/mass;
        yd = 0.005*(ydd+ydd_)*0.5 + yd_;
        y = 0.005*(yd+yd_)*0.5 + y_;

	zdd = G + az*thrust/mass;
        zd = 0.005*(zdd+zdd_)*0.5 + zd_;
        z = 0.005*(zd+zd_)*0.5 + z_;

    
}

void dynamicModel::computeOrientation()
{
        q0_=q0;
        q1_=q1;
        q2_=q2;
        q3_=q3;

        q0d_=q0d;
        q1d_=q1d;
        q2d_=q2d;
        q3d_=q3d;

        pd_ = pd;
	p_ = p;

	qd_ = qd;
	q_ = q;

	rd_ = rd;
	r_ = r;

	pd = (1/ixx)*taux;
        p = 0.005*(pd+pd_)*0.5 + p_;

	qd = (1/iyy)*tauy;
        q = 0.005*(qd+qd_)*0.5 + q_;

	rd = (1/izz)*tauz;
        r = 0.005*(rd+rd_)*0.5 + r_;

        q0d = 0.5*(-q1*p-q2*q-q3*r) ;
        q1d =0.5*(q0*p-q3*q+q2*r);
        q2d =0.5*(q3*p+q0*q-q1*r);
        q3d =0.5*(-q2*p+q1*q+q0*r);

        //std::cout<<"q1d : "<<q1d<<std::endl;
        //std::cout<<"p : "<<p<<std::endl;

        q0 = 0.005*(q0d+q0d_)*0.5 + q0_;
        q1 = 0.005*(q1d+q1d_)*0.5 + q1_;
        q2 = 0.005*(q2d+q2d_)*0.5 + q2_;
        q3 = 0.005*(q3d+q3d_)*0.5 + q3_;

        float n = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);

        q0 = q0/n;
        q1 = q1/n;
        q2 = q2/n;
        q3 = q3/n;

	//orientation.setEuler(phi, theta, psi);
	//orientation.normalize();

	
}

void dynamicModel::computeRMatrix()
{
        ax = 2*q0*q2+2*q1*q3;
        ay = -2*q0*q1+2*q2*q3;
        az = q0*q0-q1*q1-q2*q2+q3*q3;
}

void dynamicModel::sendPose()
{
    	//on publie la pose calculée qui sera ensuite interprétée par gazebo
    	poseMsgOut.header.stamp = ros::Time::now();
	poseMsgOut.pose.position.x=x;
	poseMsgOut.pose.position.y=y;  
   	poseMsgOut.pose.position.z=z;
        poseMsgOut.pose.orientation.x=q1;
        poseMsgOut.pose.orientation.y=q2;
        poseMsgOut.pose.orientation.z=q3;
        poseMsgOut.pose.orientation.w=q0;
        //orientation.setRPY(phi,theta,psi);


        //tf2::convert(orientation,poseMsgOut.pose.orientation);

   	posePub.publish(poseMsgOut);
}

void dynamicModel::sendTwist()
{
    	twistMsgOut.header.stamp = ros::Time::now();
	twistMsgOut.twist.linear.x=xd;
	twistMsgOut.twist.linear.y=yd;  
   	twistMsgOut.twist.linear.z=zd;
	twistMsgOut.twist.angular.x=p;
	twistMsgOut.twist.angular.y=q;
	twistMsgOut.twist.angular.z=r;
   	twistPub.publish(twistMsgOut);
}

void dynamicModel::spinModel()
{
    //on va appeler les fonctions au fur et à mesure
    computeOrientation();
    computeRMatrix();
    computePosition();
    sendPose();
    sendTwist();
}

int main(int argc, char**argv)
{
        ros::init(argc, argv, "dynamic_model");
        ros::NodeHandle nh;
	
	//On suppose que le drone admet deux plans de symétrie. I est donc diagonale
        float m = 1.066;//cf masse control.h
	float ixx = 0.1152;
	float iyy = 0.1152;
	float izz = 0.218;

        dynamicModel drone(nh,m,ixx,iyy,izz);

        ros::Rate Rate(200);
	
        while(ros::ok)
        {
                drone.spinModel();
                Rate.sleep();
                ros::spinOnce();
        }
}

