#ifndef VO_UTILITY
#define VO_UTILITY

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ros/ros.h>
#include "vo_core/keyframe.h"

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
using namespace cv;

class VO_Util
{
  public:
    VO_Util();

    // ------------------------------
    // --- ROS Parameters Handler ---
    // ------------------------------
    // Parameters for camera caliberation files
    string cam_cali_path_L = "";
    string cam_cali_path_R = "";
    bool rectImgInputsId = false; //true for rect image inputs

    // CPU parameters
    int CPUMultiThreadNo = 2;

    // Disparity Computing parameters
    double Epipolar_range = 1.0; // in pixel
    double MaxDepth = 20; // meter
    double MinDepth = 0.01; // meter

    // Parameters setting for feature matching
    int GFTT_maxCorners = 500;
    double GFTT_qualityLevel = 0.01;
    double GFTT_minDistance = 9;
    int GFTT_blockSize = 9;
    bool GFTT_used = false;

    float ORB_scaleFactor = 1.2f;
    int ORB_nfeatures = 1000;
    int ORB_nlevels = 8;
    int ORB_edgeThreshold = 31;
    int ORB_patchSize  = 31;

    // Parameters for Tracking performance
    float Track_distDevScale = 0.4; //Scale factor to determine the std of norm of feature matching for tracking
    float track_max_norm = 2.0;     //The max value for tracking norm (rotation*2pi + translation in m)
    float KF_update_norm = 0.25;    //For new KF: The value of norm which trigger new KF generating
    int KF_min_goodMatches = 40;    //For new KF: The min No of PnP inliner
    int KF_min_inlier = 30;         //For new KF: The min No of good features matching for tracking
    int disparity_matches = 30;
    int track_min_goodMatches = 20; //For tracking: the min No of good feature matching
    int track_min_inlier = 15;      //For tracking: the min No of PnP inlier
    int RANSAC_iterationsCount = 100;
    float RANSAC_reprojectionError = 2;
    float RANSAC_minInliersFactor = 0.8; //For solvePnP: the scale factor to determine the min No of Inliers (*goodmatches)
    int looseTrackingRestart = 3;
    int solvePnPmethod = 0;

    // ROS Topic Parameters
    string parent_frame = "VO_map";
    string child_frame = "base_link";

    // Debug Parameters
    bool DEBUG_MODE = false;
    bool SHOW_WINS = true;
    bool DRAW_DISP = true;
    bool DRAW_TRAC = true;
    bool SHOW_INFO = true;

    // --------------------------
    // --- General parameters ---
    // --------------------------
    const string FourImgWin = "Stereo images";
    bool INIT_loop_down = false;
    int looseTrackingCount = 0;
    int RestartCount = 0;

    cv::Ptr<GFTTDetector> Ptr_gftt;
    cv::Ptr<ORB> Ptr_orb;

    vector<KeyFrame> KeyFrame_L; // A vector of object for feature points extracted from the left (Tracking)
    KeyFrame prevKF_L; // To store previous KeyFrame object (left side)

    // --- Function Declaration ---
    double normofTransform( Mat rvec, Mat tvec );
    bool Cam2Imu_CVMat2Eigen(Mat &InputCVMat, Quaterniond &OutputEigen_q, Vector3d &OutputEigen_t);
};

VO_Util::VO_Util()
{
  // Initialize the feature obj
  Ptr_gftt = cv::Ptr<GFTTDetector>(new GFTTDetector(GFTT_maxCorners, GFTT_qualityLevel, GFTT_minDistance, GFTT_blockSize));
  Ptr_orb = cv::Ptr<ORB>(new ORB(ORB_nfeatures, ORB_scaleFactor, ORB_nlevels, ORB_edgeThreshold, 0, 2, ORB::HARRIS_SCORE, ORB_patchSize));

  // ---------------------
  // ROS Parameter handler
  // ---------------------
  ros::NodeHandle nh_("~"); // For parameters
  // Disparity Computing parameters
  nh_.param("Epipolar_range",  Epipolar_range, Epipolar_range ); //in Pixels
  nh_.param("MaxDepth",        MaxDepth,       MaxDepth ); //meter
  nh_.param("MinDepth",        MinDepth,       MinDepth ); //meter

  // Parameters setting for feature matching
  nh_.param("GFTT_maxCorners",    GFTT_maxCorners,   GFTT_maxCorners );
  nh_.param("GFTT_qualityLevel",  GFTT_qualityLevel, GFTT_qualityLevel );
  nh_.param("GFTT_minDistance",   GFTT_minDistance,  GFTT_minDistance );
  nh_.param("GFTT_blockSize",     GFTT_blockSize,    GFTT_blockSize );
  nh_.param("GFTT_used",          GFTT_used,         GFTT_used );

  nh_.param("ORB_scaleFactor",   ORB_scaleFactor,    ORB_scaleFactor );
  nh_.param("ORB_nfeatures",     ORB_nfeatures,      ORB_nfeatures );
  nh_.param("ORB_nlevels",       ORB_nlevels,        ORB_nlevels );
  nh_.param("ORB_edgeThreshold", ORB_edgeThreshold,  ORB_edgeThreshold );
  nh_.param("ORB_patchSize",     ORB_patchSize,      ORB_patchSize );

  // Parameters for Tracking performance
  nh_.param("Track_distDevScale",   Track_distDevScale,   Track_distDevScale );
  nh_.param("track_max_norm",       track_max_norm,       track_max_norm );
  nh_.param("KF_update_norm",       KF_update_norm,       KF_update_norm );
  nh_.param("KF_min_inlier",        KF_min_inlier,        KF_min_inlier );
  nh_.param("KF_min_goodMatches",   KF_min_goodMatches,   KF_min_goodMatches );
  nh_.param("disparity_matches",    disparity_matches,    disparity_matches );
  nh_.param("track_min_inlier",           track_min_inlier,          track_min_inlier );
  nh_.param("track_min_goodMatches",      track_min_goodMatches,     track_min_goodMatches );
  nh_.param("RANSAC_iterationsCount",     RANSAC_iterationsCount,    RANSAC_iterationsCount );
  nh_.param("RANSAC_reprojectionError",   RANSAC_reprojectionError,  RANSAC_reprojectionError );
  nh_.param("RANSAC_minInliersFactor",    RANSAC_minInliersFactor,   RANSAC_minInliersFactor );
  nh_.param("looseTrackingRestart",       looseTrackingRestart,      looseTrackingRestart );
  nh_.param("solvePnPmethod",             solvePnPmethod,            solvePnPmethod);

  // ROS Topic Parameters
  nh_.param<std::string>("VO/TF_parent",     parent_frame,    parent_frame );
  nh_.param<std::string>("VO/TF_child",      child_frame,      child_frame );

  // Parameters for camera caliberation files
  nh_.param<std::string>("cam_cali_path_L",     cam_cali_path_L,    cam_cali_path_L );
  nh_.param<std::string>("cam_cali_path_R",     cam_cali_path_R,    cam_cali_path_R );
  nh_.param("rectImgInputsId",  rectImgInputsId,  rectImgInputsId);

  // Parameter for CPU/OCL performance
  nh_.param("CPUMultiThreadNo",  CPUMultiThreadNo,  CPUMultiThreadNo);

  // Debug Parameters
  nh_.param("VO/DEBUG_MODE",      DEBUG_MODE,      DEBUG_MODE );
  nh_.param("VO/DRAW_DISP",       DRAW_DISP,       DRAW_DISP );
  nh_.param("VO/DRAW_TRAC",       DRAW_TRAC,       DRAW_TRAC );
  nh_.param("VO/SHOW_INFO",       SHOW_INFO,       SHOW_INFO );
  nh_.param("VO/SHOW_WINS",       SHOW_WINS,       SHOW_WINS );

  if(SHOW_INFO)
  {
    cout << "\n--- Camera Parameters Loading ---" << endl;
    cout << "cam_cali_path_L: " << cam_cali_path_L << endl;
    cout << "cam_cali_path_R: " << cam_cali_path_L << endl;
    cout << "VO/TF_parent: " << parent_frame << endl;
    cout << "VO/TF_child: " << child_frame << endl;
    cout << "CPUMultiThreadNo: " << CPUMultiThreadNo << endl;
    cout << "looseTrackingRestart: " << cam_cali_path_L << endl;
  }
}

double VO_Util::normofTransform( Mat rvec, Mat tvec )
{
    return fabs(min(norm(rvec), 2*CV_PI-norm(rvec)))+ fabs(norm(tvec));
}

bool VO_Util::Cam2Imu_CVMat2Eigen(Mat &InputCVMat, Quaterniond &OutputEigen_q, Vector3d &OutputEigen_t)
{
  if(InputCVMat.empty())
  {
    cout << "\nThe input Cam2Imu CVMat is empty !" << endl;
    return false;
  }

  Map<Matrix<double, Dynamic, Dynamic, RowMajor>> Eigen_Affline(InputCVMat.ptr<double>(), 4, 4);
  Matrix4d T_cam2imu = Eigen_Affline;
  OutputEigen_q = T_cam2imu.block<3,3>(0,0); //Block size (p,q), starting at (i,j)
  OutputEigen_q.normalize();
  OutputEigen_t = T_cam2imu.block<3,1>(0,3); //Block size (p,q), starting at (i,j)
  return true;
}

#endif // VO_UTILITY
