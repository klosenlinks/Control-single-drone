//ROS
#include <ros/ros.h>

//Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

//Math
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Gravity constant
#define G -9.81

class controller{

public:

	controller(const ros::NodeHandle& n); //Constructor
	~controller(){}; //Destructor
	void spinController();
        void sendToDrone();

private:

	ros::NodeHandle nh;

	//Outputs
        std_msgs::Float64 thrustMsgOut;
        std_msgs::Float64 tauxMsgOut;
        std_msgs::Float64 tauyMsgOut;
        std_msgs::Float64 tauzMsgOut;
	
	double thrust=0;
	double taux=0;
	double tauy=0;
	double tauz=0;

	//Publishers
	ros::Publisher thrustPub;
	ros::Publisher tauxPub;
	ros::Publisher tauyPub;
	ros::Publisher tauzPub;

	//Subscribers
	ros::Subscriber poseSub;
	ros::Subscriber twistSub;
	ros::Subscriber desiredposeSub;

	//Measures obtained from subscribers
	//Orientation
        tf2::Quaternion orientation_q  = tf2::Quaternion(0,0,0,1);
	//Desired orientation        
	tf2::Quaternion orientation_qdes_ = tf2::Quaternion(0,0,0,1);
	//Pose
	double x=0;
	double y=0;
	double z=0;
	//Linear velocity
	double xd=0;
	double yd=0;
	double zd=0;
	//Angular velocity
	double p = 0;
	double q = 0;
	double r = 0;
	
	//Error values and integrals
	double errx=0;
	double interrx=0;
	double erry=0;
	double interry=0;
	double errz=0;
	double interrz=0;

	//Previous values used for integration
	double errx_=0;
	double interrx_=0;
	double erry_=0;
	double interry_=0;
	double errz_=0;
	double interrz_=0;

	//Desired values
	double xdes=0;
	double ydes=0;
	double zdes=0;

	double xddes=0;
	double yddes=0;
	double zddes=0;

	double xdddes=0;
	double ydddes=0;
	double zdddes=0;

	tf2::Quaternion orientation_qdes;
	
	//Global forces
	double fx=0;
	double fy=0;
	double fz=0;

	//Orientation computation
	tf2::Vector3 xdrone;
	tf2::Vector3 ydrone;
	tf2::Vector3 zdrone;
	tf2::Vector3 dir;
	tf2::Matrix3x3 R;

	double errorqx=0;
	double errorqy=0;
	double errorqz=0;

	double errorqx_=0;
	double errorqy_=0;
	double errorqz_=0;

	double errorqxd;
	double errorqyd;
	double errorqzd;

        double qd1des;
        double qd2des;
        double qd3des;
        double qd4des;

	double pdes;
	double qdes;
	double rdes;

	//Drone parameters
	//Mass
	double m=1.066;
	//Inertia
	double Ixx=0.1152;
	double Iyy=0.1152;
	double Izz=0.218;
	
	//Sliding variables
	double sx=0;
	double sy=0;
	double sz=0;
	double signSx=0;
	double signSy=0;
	double signSz=0;
	
	//Gains 
	//PID parameters
	double kpf=3; 
	double kif=1.5; 
	double kdf=2.5; 
	//Sliding Mode parameters
	double lambda1=6;
	double lambda2=6;
	double lambda3=0.5;
	double k1=1;
	double k2=1;
	double k3=0.1;

	//Messages
	geometry_msgs::PoseStamped poseMsgIn;
	geometry_msgs::TwistStamped twistMsgIn;
	geometry_msgs::PoseStamped desiredposeMsgIn;
	
	//Callbacks
	void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void twistCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void desiredposeCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);

	//Other functions
	void computeThrust();
	void computeGlobalForces();
	void computeQdes();
	void computeQerr();
	void computeQerrd();
	void computeQddes();
	void computeOmegades();
	void computeSignS();
	void computeTorques();
};

double saturation(double x, double min, double max);
