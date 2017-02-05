#ifndef STATE_H
#define STATE_H

// Eigen
#include <Eigen/Dense>

using namespace Eigen;

struct ESKF_State
{
  // State: p, v, q, ba, bw
  Vector4d q = Vector4d(1,0,0,0);   // w + xi + jy + zk (Hamilton definition)
  Vector3d p = Vector3d::Zero();
  Vector3d v = Vector3d::Zero();
  Vector3d bw = Vector3d::Zero();  // bias of gyroscope
  Vector3d ba = Vector3d::Zero();  // bias of accelerometer

  // Reading
  Vector3d am = Vector3d::Zero();  // measurement of accelerometer
  Vector3d wm = Vector3d::Zero();  // measurement of gyroscope
  Matrix<double, 15, 15> P; // state covariance matrix
  uint sequence = -1;    //
  double timestamp = -1; // in sec
};

#endif // STATE_H
