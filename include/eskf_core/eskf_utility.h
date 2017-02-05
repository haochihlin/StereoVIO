#ifndef ESKF_UTILITY
#define ESKF_UTILITY

#include <ros/ros.h>
#include <iostream>
#include <vector>

// Eigen
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class ESKF_UTIL
{
  public:
    ESKF_UTIL();

    // Matrix for Propagation
    Matrix<double, 15, 15> Fd;
    Matrix<double, 15, 12> Gc;
    Matrix<double, 12, 12> Qd;

    // Matrix for Update
    Quaterniond Cam2Imu_q;
    Vector3d Cam2Imu_t;
    Matrix<double, 6, 15> H;

    bool INIT_id = false;
    bool Trigger_id = false; // the index to execute the whole ESKF program will be triggered by the first successful measurement update

    const Vector3d gravity = (VectorXd(3) << 9.80665, 0.0, 0.0).finished();
    const double imuReadingTimeMax = 0.06; // sec
    const double imuReadingAccelMax = 50.0; // m/sec^2
    const double imuReadingGyroMax = 50.0; // rad/sec
    const double updateTimeMax = 0.5; // sec

    inline Matrix3d Skew(const Vector3d &vec);
    inline Matrix4d QuatMat_L(const Vector4d &q);
    inline Matrix4d QuatMat_R(const Vector4d &q);
    inline void QuatNormal(Vector4d &quat);
    inline Matrix3d Quat2Rot(const Vector4d &quat);
    inline Quaterniond Vec2Quad(const Vector4d &quat);

    // Custom Parameters
    double accelerometer_noise_density = 2.0000e-3; //na
    double accelerometer_random_walk = 3.0000e-3; //nba
    double gyroscope_noise_density = 1.6968e-04; //nw
    double gyroscope_random_walk = 1.9393e-05; //nbw
    double initial_process_covariance = 1e-8; //Initial guess for P
    double stateBufferSizeInSec = 60; // How old of state obj will be eliminated (in sec)

    string ImuSubTopic = "/imu0";
    string TF_parent = "world";
    string TF_child = "eskf_base";
    bool SHOW_Info = true;
    bool DEBUG_Mode = false;

    bool UsingFixedVOCovariance = true;
    double vo_fixedstd_q = 1e-4;
    double vo_fixedstd_p = 1e-5;
};

ESKF_UTIL::ESKF_UTIL()
{
  this->Fd = MatrixXd::Identity(15, 15);
  this->Qd = MatrixXd::Identity(12, 12);
  this->Gc = MatrixXd::Zero(15, 12);
  this->Gc.block<12,12>(3,0) = MatrixXd::Identity(12, 12); //Block of size (p,q), starting at (i,j)

  this->H = MatrixXd::Zero(6, 15);
  this->H.block<3,3>(0,0) = MatrixXd::Identity(3, 3);
  this->H.block<3,3>(3,6) = MatrixXd::Identity(3, 3);

  this->Cam2Imu_t = Vector3d::Zero();
  this->Cam2Imu_q = Quaterniond::Identity();

  ros::NodeHandle nh_("~"); // For parameters
  nh_.param("accelerometer_noise_density",  accelerometer_noise_density, accelerometer_noise_density ); // [ m / s^2 / sqrt(Hz) ]
  nh_.param("accelerometer_random_walk",    accelerometer_random_walk,   accelerometer_random_walk );   // [ m / s^3 / sqrt(Hz) ]
  nh_.param("gyroscope_noise_density",      gyroscope_noise_density,     gyroscope_noise_density );     // [ rad / s / sqrt(Hz) ]
  nh_.param("gyroscope_random_walk",        gyroscope_random_walk,       gyroscope_random_walk );       //[ rad / s^2 / sqrt(Hz) ]
  nh_.param("initial_process_covariance",   initial_process_covariance,  initial_process_covariance );
  nh_.param("stateBufferSizeInSec",         stateBufferSizeInSec,        stateBufferSizeInSec ); // sec
  nh_.param("vo_fixedstd_q",                vo_fixedstd_q,               vo_fixedstd_q );
  nh_.param("vo_fixedstd_p",                vo_fixedstd_p,               vo_fixedstd_p );
  nh_.param("UsingFixedVOCovariance",       UsingFixedVOCovariance,      UsingFixedVOCovariance );

  // ROS Topic Parameters
  nh_.param<std::string>("ESKF/ImuSubTopic", ImuSubTopic, ImuSubTopic );
  nh_.param<std::string>("ESKF/TF_parent", TF_parent, TF_parent );
  nh_.param<std::string>("ESKF/TF_child", TF_child, TF_child );

  // General index
  nh_.param("ESKF/SHOW_INFO", SHOW_Info, SHOW_Info );
  nh_.param("ESKF/DEBUG_MODE", DEBUG_Mode, DEBUG_Mode );

  if(SHOW_Info)
  {
    cout << "\n--- ESKF Parameters Loading ---" << endl;
    cout << "accelerometer_noise_density: " << accelerometer_noise_density << endl;
    cout << "accelerometer_random_walk: " << accelerometer_random_walk << endl;
    cout << "gyroscope_noise_density: " << gyroscope_noise_density << endl;
    cout << "gyroscope_random_walk: " << gyroscope_random_walk << endl;
    cout << "stateBufferSizeInSec: " << stateBufferSizeInSec << endl;
    cout << "UsingFixedVOCovariance: " << UsingFixedVOCovariance << endl;
    cout << "vo_fixedstd_q: " << vo_fixedstd_q << endl;
    cout << "vo_fixedstd_p: " << vo_fixedstd_p << endl;
    cout << "ImuSubTopic: " << ImuSubTopic << endl;
    cout << "TF_parent: " << TF_parent << endl;
    cout << "TF_child: " << TF_child << endl << endl;
  }
}

inline Matrix3d ESKF_UTIL::Skew(const Vector3d &vec)
{
  return ((MatrixXd(3,3) <<    0.0, -vec[2],  vec[1],
                            vec[2],     0.0, -vec[0],
                           -vec[1],  vec[0],     0.0).finished());
}

inline Matrix4d ESKF_UTIL::QuatMat_L(const Vector4d &q)
{
  // left-quaternion-product matrices (Hamilton definition) => q1*q2 = [q1]_L * q2
  // [q1]_L = [0, -q1^T; q1, [q1]x]
  // q = w + xi + jy + zk (Hamilton definition)
  return ((MatrixXd(4,4) << q[0], -q[1], -q[2], -q[3],
                            q[1],  q[0], -q[3],  q[2],
                            q[2],  q[3],  q[0], -q[1],
                            q[3], -q[2],  q[1],  q[0]).finished());

}

inline Matrix4d ESKF_UTIL::QuatMat_R(const Vector4d &q)
{
  // right-quaternion-product matrices (Hamilton definition) => q1*q2 = [q2]_R * q1
  // [q2]_L = [0, -q2^T; q2, -[q2]x]
  // q = w + xi + jy + zk (Hamilton definition)
  return ((MatrixXd(4,4) << q[0], -q[1], -q[2], -q[3],
                            q[1],  q[0],  q[3], -q[2],
                            q[2], -q[3],  q[0],  q[1],
                            q[3],  q[2], -q[1],  q[0]).finished());
}

inline void ESKF_UTIL::QuatNormal(Vector4d &quat)
{
  quat = quat/(quat.norm());
  return ;
}

inline Matrix3d ESKF_UTIL::Quat2Rot(const Vector4d &quat)
{
  Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
  q.normalize();
  return (q.toRotationMatrix());
}

inline Quaterniond ESKF_UTIL::Vec2Quad(const Vector4d &quat)
{
  Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
  q.normalize();
  return q;
}

#endif // ESKF_UTILITY
