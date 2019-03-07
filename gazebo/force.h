//geometry
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

//gazebo
#include <gazebo_msgs/ApplyBodyWrench.h>

//URDF parameters
std::string body_name;
std::string reference_frame;
geometry_msgs::Point referencePoint;

//Outputs
geometry_msgs::Vector3 force;
geometry_msgs::Wrench wrench;	

//messages
geometry_msgs::PoseStamped poseMsgOut;
geometry_msgs::TwistStamped twistMsgOut;
gazebo_msgs::ApplyBodyWrench applyBodyWrench;

//Services
ros::ServiceClient applyBodyWrenchClient;

//functions
void sendForce();
