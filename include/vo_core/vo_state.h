#ifndef VO_STATE
#define VO_STATE

#include <opencv2/imgproc/imgproc.hpp>
// Eigen
#include <Eigen/Dense>

using namespace cv;

struct VOFrame
{
  Mat curr_grey_L, curr_grey_R, curr_bgr_L, curr_bgr_R; // Current Frame
  Mat prev_grey_L, prev_grey_R, prev_bgr_L, prev_bgr_R;// Previous Frame
};

struct VOTRack
{
  double timeStamp_curr = 0.0;
  double timeStamp_prev = 0.0;
  double timeStamp_keyframe = 0.0;
  Eigen::Quaterniond odom_q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d    odom_t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond keyframe_q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d    keyframe_t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond globalpose_q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d    globalpose_t = Eigen::Vector3d::Zero();

  Eigen::Affine3d Global_Track_A = Eigen::Affine3d::Identity();
};

#endif // VO_STATE
