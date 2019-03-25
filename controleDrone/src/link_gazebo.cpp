#include <link_gazebo.h>


//Construtor
linkGazebo::linkGazebo(const ros::NodeHandle& n): nh(n)
{
    //Subscribers
    thrustSub = nh.subscribe("/thrust",2,&linkGazebo::thrustCallBack,this);
    tauxSub = nh.subscribe("/taux",2,&linkGazebo::tauxCallBack,this);
    tauySub = nh.subscribe("/tauy",2,&linkGazebo::tauyCallBack,this);
    tauzSub = nh.subscribe("/tauz",2,&linkGazebo::tauzCallBack,this);

    //Publishers
    posePub = nh.advertise<geometry_msgs::PoseStamped>("/pose",2);
    twistPub = nh.advertise<geometry_msgs::TwistStamped>("/twist",2);

    perturbation.x=0;
    perturbation.y=2;//0.2;
    perturbation.z=0;
}

//Callbacks
void linkGazebo::thrustCallBack(const std_msgs::Float64::ConstPtr& msg)
{
    thrustMsgIn = msg->data;
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


void linkGazebo::sendForce(int i)
{
    //We call the Gazebo service ApplyBodyWrench to send a wrench to our simulation

    //We create our customer service to communicate with Gazebo
    ros::service::waitForService("/gazebo/apply_body_wrench");

    applyBodyWrenchClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    //We define here the characteristics of our body wrench
    applyBodyWrench.request.body_name = "myDrone::base_link";

    applyBodyWrench.request.reference_frame ="";//Only applies to world frame

    ros::Duration duration(1000000000);//Must be higher than the gazebo sim time
    applyBodyWrench.request.duration= duration;

    //Transition from the drone frame to the world frame
    ax = 2*q0*q2+2*q1*q3;
    ay = -2*q0*q1+2*q2*q3;
    az = q0*q0-q1*q1-q2*q2+q3*q3;

    //We compute the wrench to apply to Gazebo
    //After 10 sec of simulation, we add a perturbation
    if(i>200*10)
    {
        force.x=thrustMsgIn*ax +perturbation.x;
        force.y=thrustMsgIn*ay +perturbation.y;
        force.z=thrustMsgIn*az +perturbation.z;
    }
    else
    {
        force.x=thrustMsgIn*ax ;
        force.y=thrustMsgIn*ay ;
        force.z=thrustMsgIn*az ;
    }

    //std::cout<<force.y<<std::endl;

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
    //We retrieve the pose and twist calculated by Gazebo so that they can be used by our control nodes

    ros::service::waitForService("/gazebo/get_model_state");
    getModelStateClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    getModelState.request.model_name = "myDrone";
    getModelState.request.relative_entity_name ="world";

    getModelStateClient.call(getModelState);

    //We recover the orientation of the drone to send the force into the world frame
    q0=getModelState.response.pose.orientation.w;
    q1=getModelState.response.pose.orientation.x;
    q2=getModelState.response.pose.orientation.y;
    q3=getModelState.response.pose.orientation.z;

    //we publish the pose calculated by Gazebo
    poseMsgOut.header.stamp = ros::Time::now();
    poseMsgOut.pose = getModelState.response.pose;

    posePub.publish(poseMsgOut);

    //we publish the twist calculated by Gazebo
    twistMsgOut.header.stamp = ros::Time::now();
    twistMsgOut.twist = getModelState.response.twist;

    twistPub.publish(twistMsgOut);
}

void linkGazebo::spinModel(int i)
{
    sendForce(i);
    sendModelState();
}

/*void linkGazebo::sendPerturbation(double fx,double fy,double fz)
{
    //We call the Gazebo service ApplyBodyWrench to send a wrench to our simulation

    //We create our customer service to communicate with Gazebo
    ros::service::waitForService("/gazebo/apply_body_wrench");

    applyPerturbationClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    //We define here the characteristics of our body wrench
    applyPerturbation.request.body_name = "myDrone::base_link";

    applyPerturbation.request.reference_frame ="";//Only applies to world frame

    ros::Duration duration(1000000000);//Must be higher than the gazebo sim time
    applyPerturbation.request.duration= duration;

    //We compute the wrench to apply to Gazebo
    perturbation.x=fx;
    perturbation.y=fy;
    perturbation.z=fz;

      //  std::cout<<applyPerturbation<<std::endl;
    perturbationTorque.x=0;
    perturbationTorque.y=0;
    perturbationTorque.z=0;

    wrench.force=perturbation;
    wrench.torque=perturbationTorque;
    applyPerturbation.request.wrench=wrench;

    applyPerturbationClient.call(applyPerturbation);
}*/

int main(int argc, char**argv)
{
    ros::init(argc, argv, "link_gazebo");
    ros::NodeHandle nh;
    ros::Rate Rate(200);

    linkGazebo drone(nh);

    int i=0;
    while(ros::ok)
    {
        drone.spinModel(i);
        Rate.sleep();
        ros::spinOnce();
        i++;
        //std::cout<<i<<std::endl;
    }
}
