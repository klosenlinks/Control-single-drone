#include "trajectory.h"


trajectory::trajectory(const ros::NodeHandle& n): nh(n)
{
    //Subscriber
    poseSub = nh.subscribe("/pose",2,&trajectory::poseCallBack,this);
    desiredPoseSub = nh.subscribe("/desiredpose",2,&trajectory::desiredPoseCallBack,this);
    //Publisher
    desiredPosePub = nh.advertise<geometry_msgs::PoseStamped>("/desiredpose",2);
}

void trajectory::desiredPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    desiredPoseMsgIn = *msg;
}

void trajectory::poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = *msg;
}
void trajectory::spin()
{
    //si on est assez proche de la position, on passe à la suivante
    error.position.x = desiredPoseMsgIn.pose.position.x - pose.pose.position.x;
    error.position.y = desiredPoseMsgIn.pose.position.y - pose.pose.position.y;
    error.position.z = desiredPoseMsgIn.pose.position.z - pose.pose.position.z;

    error.orientation.w = desiredPoseMsgIn.pose.orientation.w - pose.pose.orientation.w;
    error.orientation.x = desiredPoseMsgIn.pose.orientation.x - pose.pose.orientation.x;
    error.orientation.y = desiredPoseMsgIn.pose.orientation.y - pose.pose.orientation.y;
    error.orientation.z = desiredPoseMsgIn.pose.orientation.z - pose.pose.orientation.z;

    if (norm(error) < epsilon)
    {
        sendDesiredPose(cpt);
        ROS_INFO("Nouveau setpoint");
        cpt++;
        cpt=cpt%(pose_array.poses.size());
    }
    else
    {
        sendDesiredPose(cpt);
    }
}

void trajectory::sendDesiredPose(int cpt)
{
    desiredPoseMsgOut.header.stamp = ros::Time::now();
    desiredPoseMsgOut.pose.position = pose_array.poses[cpt].position;
    desiredPosePub.publish(desiredPoseMsgOut);
}

void trajectory::generateTrajectory()
{
    //std::cout<<"taille du tableau "<<pose_array.poses.size()<<std::endl;
    float r=0.5;
    //circular sinusoïdal trajectory (you can design your own trajectory)
    for (float i=0; i<2*M_PI; i+=2*M_PI/100 )
    {
        pose_element.position.x = r*(i/3)*cos(i);
        pose_element.position.y = r*(i/3)*sin(i);
        pose_element.position.z = i/2;

        pose_array.poses.push_back(pose_element);
    }
         //on reboucle
         for (float i=2*M_PI; i>0; i-=2*M_PI/100 )
    {
        pose_element.position.x = r*(i/3)*cos(i);
        pose_element.position.y = r*(i/3)*sin(i);
        pose_element.position.z = i/2;

        pose_array.poses.push_back(pose_element);
    }
}

double norm(const geometry_msgs::Pose& p)
{
    double r;
    r=sqrt(p.position.x*p.position.x + p.position.y*p.position.y + p.position.z*p.position.z +p.orientation.w*p.orientation.w* +p.orientation.x*p.orientation.x +p.orientation.y*p.orientation.y +p.orientation.z*p.orientation.z);
    return r;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory");
    ros::NodeHandle nh;
    ros::Rate Rate(200);

    trajectory drone(nh);
    drone.generateTrajectory();
	
    while(ros::ok())
    {
        drone.spin();
        Rate.sleep();
        ros::spinOnce();
    }
}
