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
	xd = 0.005*(xdd+xdd_)/2 + xd_;
	x = 0.005*(xd+xd_)/2 + x_;

	ydd = ay*thrust/mass;
	yd = 0.005*(ydd+ydd_)/2 + yd_;
	y = 0.005*(yd+yd_)/2 + y_;

	zdd = G + az*thrust/mass;
	zd = 0.005*(zdd+zdd_)/2 + zd_;
	z = 0.005*(zd+zd_)/2 + z_;

    
}

void dynamicModel::computeOrientation()
{
	
	pd_ = pd;
	p_ = p;
	phi_ = phi;

	qd_ = qd;
	q_ = q;
	theta_ = theta;

	rd_ = rd;
	r_ = r;
	psi_ = psi;

	pd = (1/ixx)*taux;
	p = 0.005*(pd+pd_)/2 + p_;	

	qd = (1/iyy)*tauy;
	q = 0.005*(qd+qd_)/2 + q_;

	rd = (1/izz)*tauz;
	r = 0.005*(rd+rd_)/2 + r_;

	phid = cos(theta)*p + sin(theta)*r;
	thetad = sin(theta)*tan(phi)*p + q - cos(theta)*tan(phi)*r;
	psid = -1*(sin(theta)/cos(phi))*p + (cos(theta)/cos(phi))*r;

	phi = 0.005*(phid+phid_)/2 + phi_;
	theta = 0.005*(thetad+thetad_)/2 + theta_;
	psi = 0.005*(psid+psid_)/2 + psi_;

	//orientation.setEuler(phi, theta, psi);
	//orientation.normalize();

	
}

void dynamicModel::computeRMatrix()
{
	ax = cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);
	ay = sin(psi)*sin(theta)-cos(theta)*sin(phi)*cos(psi);
	az = cos(phi)*cos(theta);
}

void dynamicModel::sendPose()
{
    	//on publie la pose calculée qui sera ensuite interprétée par gazebo
    	poseMsgOut.header.stamp = ros::Time::now();
	poseMsgOut.pose.position.x=x;
	poseMsgOut.pose.position.y=y;  
   	poseMsgOut.pose.position.z=z;
	//poseMsgOut.pose.orientation.x=phi;
	//poseMsgOut.pose.orientation.y=theta;
	//poseMsgOut.pose.orientation.z=psi;
	//poseMsgOut.pose.orientation.w=0;
	orientation.setRPY(phi,theta,psi);
	tf2::convert(orientation,poseMsgOut.pose.orientation);

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
        float m = 1.477;//cf masse control.h
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

