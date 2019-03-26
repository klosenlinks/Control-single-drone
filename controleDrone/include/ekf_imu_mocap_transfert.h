#ifndef MOCAP
#define MOCAP

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "qualisys/Subject.h"

using namespace Eigen;

struct IMU_STATES{
    float Accxyz[3];
    float AngVelxyz[3];
};

struct MOCAP_STATES{
    float position[3];
    float orientation[4];
};

class ekf{

    public:
        //bias
        Vector3d b_w;
        Vector3d b_a;

        //Measured acceleration and accelero bias
        Vector3d a_0;
        Vector3d a_b;

	//quaternion
	Vector4d q;

        //Position- vitesse
        Vector3d p;
        Vector3d v;

        //Angular velocity
         Vector3d omega_m;
        //Angular velocity bias free
        Vector3d omega_bf;

        ekf();
        ~ekf(){};
        void ekfPred(IMU_STATES const &imustate);
        void ekfUpdate(qualisys::Subject mocapstate);

    private:
        
        double dt;

        MatrixXd P;
        MatrixXd Q;
        MatrixXd R;
        int n;
        short firstInit;
};

#endif
