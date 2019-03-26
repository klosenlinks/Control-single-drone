#include "ekf_imu_mocap_transfert.h"

ekf::ekf()
{
  q = Vector4d::Zero();
  // Constante de temps à modifier en fonction
  dt=1.00/200;
  n=0;
  P=MatrixXd(16,16);
  Q=MatrixXd(12,12);
  R=MatrixXd(7,7);
  b_w<<0,0,0;
  b_a<<0,0,0;
  q<<1,0,0,0;
  p<<0,0,0;
  v<<0,0,0;
  // Covariance on the state estimator
  P<<MatrixXd::Identity(16,16)*0.001;
    // Covariance gyro, accelero and gyro bias,accelero bias
  Q<<0.01*0.01/12.0,0,0,0,0,0,0,0,0,0,0,0,
     0,0.01*0.01/12.0,0,0,0,0,0,0,0,0,0,0,
     0,0,0.01*0.01/12.0,0,0,0,0,0,0,0,0,0,
     0,0,0,1*1/12.0,0,0,0,0,0,0,0,0,
     0,0,0,0,1*1/12.0,0,0,0,0,0,0,0,
     0,0,0,0,0,1*1/12.0,0,0,0,0,0,0,
     0,0,0,0,0,0,0.0001*0.0001,0,0,0,0,0,
     0,0,0,0,0,0,0,0.0001*0.0001,0,0,0,0,
     0,0,0,0,0,0,0,0,0.0001*0.0001,0,0,0,
     0,0,0,0,0,0,0,0,0,0.001*0.001,0,0,
     0,0,0,0,0,0,0,0,0,0,0.001*0.001,0,
     0,0,0,0,0,0,0,0,0,0,0,0.001*0.001;
  // Covariance position and the angle of the mocap system
  R<<0.001*0.001/12.0,0,0,0,0,0,0,
     0,0.001*0.001/12.0,0,0,0,0,0,
     0,0,0.001*0.001/12.0,0,0,0,0,
     0,0,0,0.002*0.002/12.0,0,0,0,
     0,0,0,0,0.002*0.002/12.0,0,0,
     0,0,0,0,0,0.002*0.002/12.0,0,
     0,0,0,0,0,0,0.002*0.002/12.0;
    firstInit=0;
}

void ekf::ekfPred(IMU_STATES const &imustate)
{
    Vector3d a,aw;
    a<<imustate.Accxyz[0],imustate.Accxyz[1],imustate.Accxyz[2];

    omega_m<<imustate.AngVelxyz[0],imustate.AngVelxyz[1],imustate.AngVelxyz[2];

    //Prediction part
    MatrixXd Omega(4,3);
    Omega<<-q[1],-q[2],-q[3], q[0],-q[3],q[2], q[3],q[0],-q[1], -q[2],q[1],q[0];
    double norm_q;

    //Rotation matrix from inertial frame to body
    Matrix3d Rbe;

    Rbe<<pow(q[0],2)+pow(q[1],2)-pow(q[2],2)-pow(q[3],2),2*(q[1]*q[2]+q[0]*q[3]),2*(q[1]*q[3]-q[0]*q[2]),
    2*(q[1]*q[2]-q[0]*q[3]),pow(q[0],2)-pow(q[1],2)+pow(q[2],2)-pow(q[3],2),2*(q[2]*q[3]+q[0]*q[1]),
    2*(q[1]*q[3]+q[0]*q[2]),2*(q[2]*q[3]-q[0]*q[1]),pow(q[0],2)-pow(q[1],2)-pow(q[2],2)+pow(q[3],2);

    Vector3d g;
    // Global frame, z is up
    g<<0,0,-9.807;

    //Prediction
    p=p+v*dt;
    v=v+(Rbe.transpose()*(a+b_a)+g)*dt;
    q=q+1.00/2.00*Omega*(omega_m-b_w)*dt;
    a_0=Rbe.transpose()*(a);
    a_b=Rbe.transpose()*(a+b_a);

    omega_bf=omega_m-b_w;

    //Normalize Quaternion
    norm_q=pow(pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2),0.5);
    q<<q[0]/norm_q,q[1]/norm_q,q[2]/norm_q,q[3]/norm_q;

    //P computation
    MatrixXd F(16,16);
    F<<0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 0, 0, 2.0*a[0]*q[0] - 2.0*a[2]*q[2] + 2.0*a[1]*q[3], 2.0*a[0]*q[1] + 2*a[1]*q[2] + 2.0*a[2]*q[3], -2*a[2]*q[0] + 2.0*a[1]*q[1] - 2.0*a[0]*q[2], 2.0*a[1]*q[0] + 2.0*a[2]*q[1] - 2.0*a[0]*q[3], 0, 0, 0,
    pow(q[0],2) + pow(q[1],2) - pow(q[2],2) - pow(q[3],2), 2.0*(q[1]*q[2] + q[0]*q[3]), 2.0*(-q[0]*q[2] + q[1]*q[3]),
    0, 0, 0, 0, 0, 0, 2.0*a[1]*q[0] + 2.0*a[2]*q[1] - 2.0*a[0]*q[3], 2.0*a[2]*q[0] - 2*a[1]*q[1] + 2.0*a[0]*q[2], 2*a[0]*q[1] + 2.0*a[1]*q[2] + 2.0*a[2]*q[3], -2.0*a[0]*q[0] + 2.0*a[2]*q[2] - 2.0*a[1]*q[3], 0, 0, 0,
    2.0*(q[1]*q[2] - q[0]*q[3]), pow(q[0],2) - pow(q[1],2) + pow(q[2],2) - pow(q[3],2), 2.0*(q[0]*q[1] + q[2]*q[3]),
    0, 0, 0, 0, 0, 0, 2.0*a[2]*q[0] - 2.0*a[1]*q[1] + 2.0*a[0]*q[2], -2.0*a[1]*q[0] - 2*a[2]*q[1] + 2.0*a[0]*q[3], 2*a[0]*q[0] - 2.0*a[2]*q[2] + 2.0*a[1]*q[3], 2.0*a[0]*q[1] + 2.0*a[1]*q[2] + 2.0*a[2]*q[3], 0, 0, 0,
    2.0*(q[0]*q[2] + q[1]*q[3]), 2*(-q[0]*q[1] + q[2]*q[3]), pow(q[0],2) - pow(q[1],2) - pow(q[2],2) + pow(q[3],2),
    0, 0, 0, 0, 0, 0, 0, (b_w[0] - omega_m[0])/2.00, (b_w[1] - omega_m[1])/2.00, (b_w[2] - omega_m[2])/2.00, q[1]/2.00, q[2]/2.00, q[3]/2.00,0,0,0,
    0, 0, 0, 0, 0, 0, 1.00/2.00*(-b_w[0] + omega_m[0]), 0, 1.00/2.00*(-b_w[2] + omega_m[2]), (b_w[1] - omega_m[1])/2.00, -(q[0]/2.00), q[3]/2.00, -(q[2]/2.00),0,0,0,
    0, 0, 0, 0, 0, 0, 1.00/2.00*(-b_w[1] + omega_m[1]), (b_w[2] - omega_m[2])/2.00, 0, 1.00/2.00*(-b_w[0] + omega_m[0]), -(q[3]/2.00), -(q[0]/2.00), q[1]/2.00,0,0,0,
    0, 0, 0, 0, 0, 0, 1.00/2.00*(-b_w[2] + omega_m[2]), 1.00/2.00*(-b_w[1] + omega_m[1]), (b_w[0] - omega_m[0])/2.00, 0, q[2]/2.00, -(q[1]/2.00), -(q[0]/2.00),0,0,0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0;

    MatrixXd G(16,12);
    G<<0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, pow(q[0],2) + pow(q[1],2) - pow(q[2],2) - pow(q[3],2), 2.0*(q[1]*q[2] + q[0]*q[3]), 2.0*(-q[0]*q[2] + q[1]*q[3]), 0, 0, 0,0,0,0,
    0, 0, 0, 2.0*(q[1]*q[2] - q[0]*q[3]), pow(q[0],2) - pow(q[1],2) + pow(q[2],2) - pow(q[3],2), 2.0*(q[0]*q[1] + q[2]*q[3]), 0, 0, 0,0,0,0,
    0, 0, 0, 2.0*(q[0]*q[2] + q[1]*q[3]), 2*(-q[0]*q[1] + q[2]*q[3]), pow(q[0],2) - pow(q[1],2) - pow(q[2],2) + pow(q[3],2), 0, 0, 0,0,0,0,
    -(q[1]/2.00), -(q[2]/2.00), -(q[3]/2.00), 0, 0, 0, 0, 0, 0,0,0,0,
    q[0]/2.00, -(q[3]/2.00), q[2]/2.00, 0, 0, 0, 0, 0, 0,0,0,0,
    q[3]/2.00, q[0]/2.00, -(q[1]/2.00), 0, 0, 0, 0, 0, 0,0,0,0,
    -(q[2]/2.00), q[1]/2.00, q[0]/2.00, 0, 0, 0, 0, 0, 0,0,0,0,
    0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0;

    P=(MatrixXd::Identity(16,16)+F*dt)*P*(MatrixXd::Identity(16,16)+F*dt).transpose()+pow(dt,2)*G*Q*G.transpose();
}


void ekf::ekfUpdate(qualisys::Subject mocapstate)
{
  if (firstInit>0)
  {
      //Update part
      MatrixXd H(7,16);

      H<<1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
         0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
         0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,
         0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0,0,0,0,
         0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0,0,0,0,
         0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0,0,0,0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0,0,0,0;

      VectorXd error(7);
      error<<p[0]-mocapstate.position.x,
             p[1]-mocapstate.position.y,
             p[2]-mocapstate.position.z,
             q[0]-mocapstate.orientation.w,
             q[1]-mocapstate.orientation.x,
             q[2]-mocapstate.orientation.y,
             q[3]-mocapstate.orientation.z;

      MatrixXd K(16,7);
      K=P*H.transpose()*(H*P*H.transpose()+R).inverse();

      //Update
      VectorXd x(16);
      x<<p[0],p[1],p[2],v[0],v[1],v[2],q[0],q[1],q[2],q[3],b_w[0],b_w[1],b_w[2],b_a[0],b_a[1],b_a[2];
      x=x-K*error;

      p<<x[0],x[1],x[2];
      v<<x[3],x[4],x[5];
      q<<x[6],x[7],x[8],x[9];
      b_w<<x[10],x[11],x[12];
      b_a<<x[13],x[14],x[15];
      //Angular velocities
      omega_bf=omega_m-b_w;
      P=P-K*H*P;
    }

    else
    {
      p<<mocapstate.position.x,mocapstate.position.y,mocapstate.position.z;
      q<<mocapstate.orientation.w,mocapstate.orientation.x,mocapstate.orientation.y,mocapstate.orientation.z;
      firstInit++;
	ROS_INFO("%f,%f,%f",p[0],p[1],p[2]);
    }

    double norm_q;
    //Normalize Quaternion
    norm_q=pow(pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2),0.5);
    q<<q[0]/norm_q,q[1]/norm_q,q[2]/norm_q,q[3]/norm_q;
}
