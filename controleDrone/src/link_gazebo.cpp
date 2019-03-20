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

    applyBodyWrench.request.reference_frame ="myDrone::base_link";

    ros::Duration five_seconds(0.005);
    applyBodyWrench.request.duration= five_seconds;

    force.x=0;
    force.y=0;
    force.z=thrustMsgIn;

    torque.x=tauxMsgIn;
    torque.y=tauyMsgIn;
    torque.z=tauzMsgIn;

    //std::cout<<force.x<<" ; "<<force.y<<" ; "<<force.z<<" ; "<<torque.x<<" ; "<<torque.y<<" ; "<<torque.z<<" ; "<<std::endl;

    wrench.force=force;
    wrench.torque=torque;
    applyBodyWrench.request.wrench=wrench;

    std::cout<<wrench.force<<std::endl;

    applyBodyWrenchClient.call(applyBodyWrench);
}

void linkGazebo::sendModelState()
{
    ros::service::waitForService("/gazebo/get_model_state");
    getModelStateClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    getModelState.request.model_name = "myDrone";
    getModelState.request.relative_entity_name ="world";

    getModelStateClient.call(getModelState);

    //on publie la pose calculée par Gazebo
    poseMsgOut.header.stamp = ros::Time::now();
    poseMsgOut.pose = getModelState.response.pose;//response

    posePub.publish(poseMsgOut);

    //on publie le twist calculé par Gazebo
    twistMsgOut.header.stamp = ros::Time::now();
    twistMsgOut.twist = getModelState.response.twist;//response

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
