#include "trajectory.h"


trajectory::trajectory(const ros::NodeHandle& n): nh(n)
{
    //Publisher
    desiredPosePub = nh.advertise<geometry_msgs::PoseStamped>("/desiredpose",2);
}

void trajectory::spin()
{
    //We publish the desired pose and we move to the next one
    sendDesiredPose(cpt);
    cpt++;
    cpt=cpt%(pose_array.poses.size());
}

void trajectory::sendDesiredPose(int cpt)
{
    desiredPoseMsgOut.header.stamp = ros::Time::now();
    desiredPoseMsgOut.pose.position = pose_array.poses[cpt].position;
    desiredPosePub.publish(desiredPoseMsgOut);
}

void trajectory::generateTrajectory()
{
   //circular upward trajectory of radius r
    float r=1;

    for (float i=0; i<1; i+=0.0005 )
    {
        pose_element.position.x = 0;
        pose_element.position.y = 0;
        pose_element.position.z = i;

        pose_array.poses.push_back(pose_element);
    }

    for (float i=0; i<1; i+=0.0005 )
    {
        pose_element.position.x = r*i;
        pose_element.position.y = 0;
        pose_element.position.z = 1;

        pose_array.poses.push_back(pose_element);
    }

    for (float i=0; i<6*M_PI; i+=6*M_PI/20000 )
    {
        pose_element.position.x = r*cos(i);
        pose_element.position.y = r*sin(i);
        pose_element.position.z = 1+i/10;

        pose_array.poses.push_back(pose_element);
    }
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
