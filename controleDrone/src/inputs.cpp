#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "ekf_imu_mocap_transfert.h"

//Subscribers
ros::Subscriber mocapSub;
ros::Subscriber imuSub;

//Publishers
ros::Publisher posePub;
ros::Publisher twistPub;

//Messages
geometry_msgs::PoseStamped poseMsgOut;
geometry_msgs::TwistStamped twistMsgOut;
nav_msgs::Odometry mocapMsgIn;
sensor_msgs::Imu imuMsgIn;

//EKF
ekf kalmann;
qualisys::Subject mocapstate;
IMU_STATES imustate; 

void mocapCallBack(const nav_msgs::Odometry::ConstPtr& msg){

	mocapMsgIn = *msg;

	mocapstate.position.x = mocapMsgIn.pose.pose.position.x;
	mocapstate.position.y = mocapMsgIn.pose.pose.position.y;
	mocapstate.position.z = mocapMsgIn.pose.pose.position.z;
	mocapstate.orientation.x = mocapMsgIn.pose.pose.orientation.x;
	mocapstate.orientation.y = mocapMsgIn.pose.pose.orientation.y;
	mocapstate.orientation.z = mocapMsgIn.pose.pose.orientation.z;
	mocapstate.orientation.w = mocapMsgIn.pose.pose.orientation.w;

	kalmann.ekfupdate(mocapstate);

	//Messages Out
	poseMsgOut.pose.position.x = kalmann.p[0];
	poseMsgOut.pose.position.y = kalmann.p[1];
	poseMsgOut.pose.position.z = kalmann.p[2];
	poseMsgOut.pose.orientation.w = kalmann.q[0];
	poseMsgOut.pose.orientation.x = kalmann.q[1];
	poseMsgOut.pose.orientation.w = kalmann.q[2];
	poseMsgOut.pose.orientation.w = kalmann.q[3];
	
	twistMsgOut.twist.linear.x = kalmann.v[0];
	twistMsgOut.twist.linear.x = kalmann.v[1];
	twistMsgOut.twist.linear.x = kalmann.v[2];
	twistMsgOut.twist.angular.x = kalmann.omega_bf[0];
	twistMsgOut.twist.angular.x = kalmann.omega_bf[1];
	twistMsgOut.twist.angular.x = kalmann.omega_bf[2];

	//Publish
	posePub.publish(poseMsgOut);
	twistPub.publish(twistMsgOut);

}

void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg){
	
	imuMsgIn = *msg;
	
	imustate.Accxyz[0] = imuMsgIn.linear_acceleration.x;
	imustate.Accxyz[1] = imuMsgIn.linear_acceleration.y;
	imustate.Accxyz[2] = imuMsgIn.linear_acceleration.z;

	imustate.AngVelxyz[0] = imuMsgIn.angular_velocity.x;
	imustate.AngVelxyz[1] = imuMsgIn.angular_velocity.y;
	imustate.AngVelxyz[2] = imuMsgIn.angular_velocity.z;
	 
	kalmann.ekfPred(imuState);

	//Messages Out
	poseMsgOut.pose.position.x = kalmann.p[0];
	poseMsgOut.pose.position.y = kalmann.p[1];
	poseMsgOut.pose.position.z = kalmann.p[2];
	poseMsgOut.pose.orientation.w = kalmann.q[0];
	poseMsgOut.pose.orientation.x = kalmann.q[1];
	poseMsgOut.pose.orientation.w = kalmann.q[2];
	poseMsgOut.pose.orientation.w = kalmann.q[3];
	
	twistMsgOut.twist.linear.x = kalmann.v[0];
	twistMsgOut.twist.linear.x = kalmann.v[1];
	twistMsgOut.twist.linear.x = kalmann.v[2];
	twistMsgOut.twist.angular.x = kalmann.omega_bf[0];
	twistMsgOut.twist.angular.x = kalmann.omega_bf[1];
	twistMsgOut.twist.angular.x = kalmann.omega_bf[2];

	//Publish
	posePub.publish(poseMsgOut);
	twistPub.publish(twistMsgOut);
}


}


int main(int argc, char**argv)
{
        ros::init(argc, argv, "inputs");
        ros::NodeHandle nh;

	mocapSub = nh.subscribe("/qualisys/odom",10,&mocapCallBack,this);
	imuSub = nh.subscribe("/imustate",10,&imuCallBack,this); // topic ? /drone1/mavros/imu/data_raw ?

	posePub = nh.advertise<geometry_msgs::PoseStamped>("/pose",2);
	twistPub = nh.advertise<geometry_msgs::TwistStamped>("/twist",2);


        ros::Rate Rate(200);
	
	
        while(ros::ok)
        {
		
		ros::spinOnce();
                Rate.sleep();
              
        }
}
