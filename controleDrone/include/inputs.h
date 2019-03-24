//ROS
#include "ros/ros.h"

//Messages
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

//Kalman filter
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
qualisys::Subject mocapMsgIn;
sensor_msgs::Imu imuMsgIn;

//EKF
ekf kalmann;
IMU_STATES imustate;
