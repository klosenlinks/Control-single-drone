#include "ros/ros.h"
#include <link_gazebo.h>


//Construtor
linkGazebo::linkGazebo(const ros::NodeHandle& n): nh(n)
{
    //subscriber
    thrustSub = nh.subscribe("/thrust",2,&linkGazebo::thrustCallBack,this);
    tauxSub = nh.subscribe("/taux",2,&linkGazebo::tauxCallBack,this);
    tauySub = nh.subscribe("/tauy",2,&linkGazebo::tauyCallBack,this);
    tauzSub = nh.subscribe("/tauz",2,&linkGazebo::tauzCallBack,this);
    //publisher
    posePub = nh.advertise<geometry_msgs::PoseStamped>("/pose",2);
    twistPub = nh.advertise<geometry_msgs::TwistStamped>("/twist",2);
}

//Callbacks
void linkGazebo::thrustCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    thrustMsgIn = msg->data; //float
}

void linkGazebo::tauxCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    tauxMsgIn = msg->data;
}

void linkGazebo::tauyCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    tauyMsgIn = msg->data;
}

void linkGazebo::tauzCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    tauzMsgIn = msg->data;
}


void linkGazebo::sendForce()
{
    //on crée notre service client pour communiquer avec gazebo
    ros::service::waitForService("/gazebo/apply_body_wrench");

    applyBodyWrenchClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    //on définit ici les caractéristiques de notre body wrench

    applyBodyWrench.request.body_name = "myDrone::base_link";

    applyBodyWrench.request.reference_frame ="";//only applies to world frame

    ros::Duration duration(1000000000);
    applyBodyWrench.request.duration= duration;

    //passage du repère du drone au repère monde (à cause de gazebo qui envoie la force que dans le repère monde)
    ax = 2*q0*q2+2*q1*q3;
    ay = -2*q0*q1+2*q2*q3;
    az = q0*q0-q1*q1-q2*q2+q3*q3;

    force.x=thrustMsgIn*ax;
    force.y=thrustMsgIn*ay;
    force.z=thrustMsgIn*az;

    torque.x=tauxMsgIn;
    torque.y=tauyMsgIn;
    torque.z=tauzMsgIn;

    wrench.force=force;
    wrench.torque=torque;
    applyBodyWrench.request.wrench=wrench;

    applyBodyWrenchClient.call(applyBodyWrench);
}

void linkGazebo::sendModelState()
{
    ros::service::waitForService("/gazebo/get_model_state");
    getModelStateClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    getModelState.request.model_name = "myDrone";
    getModelState.request.relative_entity_name ="world";

    getModelStateClient.call(getModelState);

    //On récupère l'orientation du drone pour envoyer la force dans le repère monde (pb de gazebo))
    q0=getModelState.response.pose.orientation.w;
    q1=getModelState.response.pose.orientation.x;
    q2=getModelState.response.pose.orientation.y;
    q3=getModelState.response.pose.orientation.z;

    //on publie la pose calculée par Gazebo
    poseMsgOut.header.stamp = ros::Time::now();
    poseMsgOut.pose = getModelState.response.pose;

    posePub.publish(poseMsgOut);

    //on publie le twist calculé par Gazebo
    twistMsgOut.header.stamp = ros::Time::now();
    twistMsgOut.twist = getModelState.response.twist;

    twistPub.publish(twistMsgOut);
}

void linkGazebo::spinModel()
{
    sendForce(); //On envoie la force calculée par le controller à gazebo
    sendModelState(); //Gazebo nous renvoie la pose du drone (position et orientation) ainsi que son twist ( vitesse linéaire et angulaire)
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "link_gazebo");
    ros::NodeHandle nh;

    linkGazebo drone(nh);

    ros::Rate Rate(200);

    while(ros::ok)
    {
        drone.spinModel();
        Rate.sleep();
        ros::spinOnce();
    }
}
