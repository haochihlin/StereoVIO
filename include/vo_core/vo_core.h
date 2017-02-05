#ifndef VO_CORE
#define VO_CORE

#include <iostream>
#include <vector>
#include <chrono> // [DEBUG] For tic-toc computing

// ROS Libs
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// OpenCV Libs
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "vo_core/vo_state.h"
#include "vo_core/vo_utility.h"
#include "vo_core/keyframe.h"
#include "vo_core/camerapare.h"

using namespace std;
using namespace cv;
using namespace Eigen;

class VO_Core
{
    public:
      VO_Core(){}
      bool stereoVOProcess(VOFrame &voFrame, VOTRack &voTrack, VO_Util &VoUtil, CameraPara &CamPara, double ImgMsgTime);
      void VOGlobalTFPub(VOTRack &voTrack, VO_Util &VoUtil);

    private:
      void drawResult(KeyFrame *currKF_R, KeyFrame *currKF_L, vector<DMatch> &matches, vector<DMatch> &goodMatches, Mat &inliers, VOFrame &voFrame, VO_Util &VoUtil, CameraPara &CamPara);

};

void VO_Core::VOGlobalTFPub(VOTRack &voTrack, VO_Util &VoUtil)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  //tf::poseEigenToTF(this->voTrack.Global_Track_A, transform);
  tf::Quaternion q;
  tf::Vector3 vec;
  tf::quaternionEigenToTF(voTrack.globalpose_q, q);
  tf::vectorEigenToTF(voTrack.globalpose_t, vec);
  transform.setOrigin(vec);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), VoUtil.parent_frame, VoUtil.child_frame));

}


void VO_Core::drawResult(KeyFrame *currKF_R, KeyFrame *currKF_L, vector<DMatch> &matches, vector<DMatch> &goodMatches, Mat &inliers, VOFrame &voFrame, VO_Util &VoUtil, CameraPara &CamPara)
{
  // --- Draw keypoints ---
  drawKeypoints(voFrame.curr_grey_L, currKF_L->ori_kp, voFrame.curr_bgr_L, Scalar(0,255,0));
  drawKeypoints(voFrame.curr_grey_R, currKF_R->ori_kp, voFrame.curr_bgr_R, Scalar(0,255,0));

  // --- Construct a matrix including four images ---
  // currL | currR
  // =============
  // prevL | prevR
  Mat prevHconcat, currHconcat, FourImgResut;
  // --- Combine two previous images (H) ---
  hconcat(voFrame.prev_bgr_L, voFrame.prev_bgr_R, prevHconcat);
  // --- Combine two current images (H)---
  hconcat(voFrame.curr_bgr_L, voFrame.curr_bgr_R, currHconcat);
  // --- Combine two H matrices (V)

  vconcat(currHconcat, prevHconcat, FourImgResut);

  // --- Drawing the current disparity mathcing result ---
  if(VoUtil.DRAW_DISP)
  {
      vector<Point2f>::const_iterator i_R = currKF_R->kp_2f.begin();// Draw matching line
      for (vector<Point2f>::const_iterator i_L = currKF_L->kp_2f.begin(); i_L != currKF_L->kp_2f.end(); ++i_L)
      {
          line(FourImgResut, Point2f(i_L->x, i_L->y), Point2f(i_R->x + CamPara.width, i_R->y), Scalar(0,0,255));
          i_R++;
      }
      if(VoUtil.DEBUG_MODE)
          cout << "[DEBUG] Disparity matching result: " << currKF_R->kp_2f.size() << endl;
  }

  // --- Drawing the tracking mathcing result (ori: blue) ---
  if(VoUtil.DRAW_TRAC)
      for(vector<DMatch>::const_iterator i = matches.begin(); i != matches.end(); ++i)
          line(FourImgResut, (VoUtil.prevKF_L.kp_2f[i->queryIdx] + Point2f(0,CamPara.height)), currKF_L->kp_2f[i->trainIdx], Scalar(255,0,0));

  // --- Drawing the tracking mathcing result (good: green) ---
  if(VoUtil.DRAW_TRAC)
      for(vector<DMatch>::const_iterator i = goodMatches.begin(); i != goodMatches.end(); ++i)
          line(FourImgResut, (VoUtil.prevKF_L.kp_2f[i->queryIdx] + Point2f(0,CamPara.height)), currKF_L->kp_2f[i->trainIdx], Scalar(0,255,0));

  // --- Drawing the tracking mathcing result (inliner: red) ---
  for(int i=0; i<inliers.rows; i++)
  {
      if(VoUtil.DRAW_TRAC)
          line(FourImgResut, (VoUtil.prevKF_L.track_kp_2f[inliers.ptr<int>(i)[0]] + Point2f(0,CamPara.height)), currKF_L->track_kp_2f[inliers.ptr<int>(i)[0]], Scalar(0,0,255), 3);
      if(VoUtil.DEBUG_MODE)
          cout << "[DEBUG] 3D keypoint: " << setw(12) << VoUtil.prevKF_L.track_kp_3f[inliers.ptr<int>(i)[0]].x << setw(12) <<
                                                         VoUtil.prevKF_L.track_kp_3f[inliers.ptr<int>(i)[0]].y << setw(12) <<
                                                         VoUtil.prevKF_L.track_kp_3f[inliers.ptr<int>(i)[0]].z << setw(1) << endl << endl;
  }

  // --- Update GUI Window ---
  imshow(VoUtil.FourImgWin, FourImgResut);
  waitKey(1);
}

bool VO_Core::stereoVOProcess(VOFrame &voFrame, VOTRack &voTrack, VO_Util &VoUtil, CameraPara &CamPara, double ImgMsgTime)
{
  // =========================
  // --- Feature Disparity ---
  // =========================
  KeyFrame *currKF_L = new KeyFrame;
  KeyFrame *currKF_R = new KeyFrame;

  // --- Set tic time ---
  auto t0 = std::chrono::high_resolution_clock::now();

  // --- Feature detect and compute ---
  VoUtil.Ptr_gftt->detect(voFrame.curr_grey_L, currKF_L->ori_kp);
  VoUtil.Ptr_gftt->detect(voFrame.curr_grey_R, currKF_R->ori_kp);
  VoUtil.Ptr_orb->compute(voFrame.curr_grey_L, currKF_L->ori_kp, currKF_L->ori_desc);
  VoUtil.Ptr_orb->compute(voFrame.curr_grey_R, currKF_R->ori_kp, currKF_R->ori_desc);

  // --- BFMatcher for disparity matching ---
  vector<DMatch> dispMatches;
  BFMatcher dispMatcher(NORM_HAMMING, true);
  dispMatcher.match(currKF_L->ori_desc, currKF_R->ori_desc, dispMatches); // queryDescriptors, trainDescriptors
  if(dispMatches.size() < VoUtil.disparity_matches)
  {
      cout << "Low disparity matching !!, size of matches: " << dispMatches.size() << endl << endl;
      delete currKF_L;
      delete currKF_R;
      VoUtil.looseTrackingCount++;
      return false; // // finish this call-back func
  }

  // --- Find "Good" Matches by using epipolar constraint ---
  vector<DMatch> dispGoodMatches;
  for ( size_t i=0; i<dispMatches.size(); i++ )
  {
      int MatchId_L = dispMatches[i].queryIdx;
      int MatchId_R = dispMatches[i].trainIdx;
      double epipolarDist = currKF_L->ori_kp[MatchId_L].pt.y - currKF_R->ori_kp[MatchId_R].pt.y;
      double disparity = currKF_L->ori_kp[MatchId_L].pt.x - currKF_R->ori_kp[MatchId_R].pt.x;
      double depth = CamPara.f_times_B/disparity; // Z=(focal*Baseline)/(disparity)
      if( abs(epipolarDist) <= VoUtil.Epipolar_range && depth <= VoUtil.MaxDepth && depth >= VoUtil.MinDepth )
      {
          dispGoodMatches.push_back( dispMatches[i] );
          // --- Update 2D keypoints and descriptors ---
          currKF_L->desc.push_back(currKF_L->ori_desc.row(MatchId_L) );
          currKF_R->desc.push_back(currKF_R->ori_desc.row(MatchId_R) );
          currKF_L->kp_2f.push_back(currKF_L->ori_kp[MatchId_L].pt);
          currKF_R->kp_2f.push_back(currKF_R->ori_kp[MatchId_R].pt);

          // --- Update 3D keypoints ---
          Point3f p_3d;
          p_3d.z = depth;
          p_3d.x = (currKF_L->ori_kp[MatchId_L].pt.x - CamPara.c_x)*p_3d.z/CamPara.focal_length;
          p_3d.y = (currKF_L->ori_kp[MatchId_L].pt.y - CamPara.c_y)*p_3d.z/CamPara.focal_length;
          currKF_L->kp_3f.push_back(p_3d);
      }
  }

  if(VoUtil.DEBUG_MODE)
    cout << "[DEBUG] dispGoodMatches: " << dispGoodMatches.size() << endl;

  // ========================
  // ------ Initial Loop ------
  // ========================
  if(!VoUtil.INIT_loop_down || VoUtil.looseTrackingCount >= VoUtil.looseTrackingRestart)
  {
      // --- Push back current KF ---
      //VoUtil.KeyFrame_L.clear();
      //VoUtil.KeyFrame_L.push_back(*currKF_L);
      VoUtil.prevKF_L = *currKF_L; // Store current KeyFrame object into "prevKF_L" object
      VoUtil.prevKF_L.ClearTrackKP();

      // --- Initialize the pose
      //this->prevKF_L.KFLocal_R = this->Global_Track_R;
      //this->prevKF_L.KFLocal_T = this->Global_Track_T;
      if(VoUtil.DEBUG_MODE)
        VoUtil.prevKF_L.KFLocal_A = voTrack.Global_Track_A;
      voTrack.keyframe_q = voTrack.globalpose_q;
      voTrack.keyframe_t = voTrack.globalpose_t;

      // --- Put back the previous image ---
      voFrame.prev_grey_L = voFrame.curr_grey_L.clone();
      voFrame.prev_grey_R = voFrame.curr_grey_R.clone();
      cvtColor(voFrame.curr_grey_L, voFrame.prev_bgr_L, CV_GRAY2BGR);
      cvtColor(voFrame.curr_grey_R, voFrame.prev_bgr_R, CV_GRAY2BGR);
      delete currKF_L;
      delete currKF_R;

      if(VoUtil.looseTrackingCount >= VoUtil.looseTrackingRestart)
      {
        cout << endl << "=== Restart ! ===" << endl << endl;
        VoUtil.RestartCount++;
      }
      else
        cout << endl << "=== Initial loop down ! ===" << endl << endl;
      VoUtil.INIT_loop_down = true;
      VoUtil.looseTrackingCount = 0;
      return false;
  }

  // ==========================
  // --- Tracking Algorithm ---
  // ==========================
  // --- BFMatcher Matching for Tracking ---
  // Matching with the previous frame
  vector<DMatch> matches;
  BFMatcher matcher(NORM_HAMMING, true);
  matcher.match(VoUtil.prevKF_L.desc, currKF_L->desc, matches); // queryDescriptors, trainDescriptors
  if(VoUtil.DEBUG_MODE)
    cout << "[DEBUG] Tracking Ori matches: " << matches.size() << endl;

  // --- Find "Good" Matches ---
  // Remove high distances matching result (no larger than 1-sigma)
  float mean, stddev, variance;
  mean = stddev = variance= 0.0;
  for(vector<DMatch>::const_iterator i = matches.begin(); i != matches.end(); ++i)
      mean = mean + i->distance;
  mean = mean/matches.size();
  for(vector<DMatch>::const_iterator i = matches.begin(); i != matches.end(); ++i)
      variance += (i->distance - mean) * (i->distance - mean);
  stddev = sqrt(variance / (matches.size()-1)); // sample standard deviation
  if(VoUtil.DEBUG_MODE)
      cout << "[DEBUG] Mean: " << mean << ", Std: " << stddev << endl;

  vector<DMatch> goodMatches; // Create a new vecor to contain good matches
  stddev = stddev*VoUtil.Track_distDevScale + mean; // Set the valve for 1-sigma distribution
  VoUtil.prevKF_L.ClearTrackKP(); // Clear all remaining kp vectors.
  for(vector<DMatch>::const_iterator i = matches.begin(); i != matches.end(); ++i)
  {
      if(i->distance <= stddev)
      {
          goodMatches.push_back(*i);
          VoUtil.prevKF_L.track_kp_3f.push_back(VoUtil.prevKF_L.kp_3f[i->queryIdx]);
          VoUtil.prevKF_L.track_kp_2f.push_back(VoUtil.prevKF_L.kp_2f[i->queryIdx]);
          currKF_L->track_kp_2f.push_back(currKF_L->kp_2f[i->trainIdx]);
          currKF_L->track_kp_3f.push_back(currKF_L->kp_3f[i->trainIdx]);
      }
  }

  // --- Test good Matches ---
  if(VoUtil.DEBUG_MODE)
    cout << "[DEBUG] Good matches for Tracking: " << goodMatches.size() << endl;
  if(goodMatches.size() < VoUtil.track_min_goodMatches)
  {
      cout << "Lose Tracking !!, size of good matches: " << goodMatches.size() << endl << endl;
      delete currKF_L;
      delete currKF_R;
      VoUtil.looseTrackingCount++;
      return false; // // finish this call-back func
  }

  // --- Solve PnP problem ---
  Mat Rodrigues_R, temp_T, inliers, temp_R; // Type for rvec or tvec: CV_64FC1
  solvePnPRansac( VoUtil.prevKF_L.track_kp_3f, currKF_L->track_kp_2f, CamPara.projMatrix, Mat(), Rodrigues_R, temp_T, false,
                  VoUtil.RANSAC_iterationsCount, VoUtil.RANSAC_reprojectionError, VoUtil.RANSAC_minInliersFactor*goodMatches.size(), inliers, VoUtil.solvePnPmethod  );

  if(VoUtil.DEBUG_MODE)
    cout<<"[DEBUG] PNP inliers: "<<inliers.rows<<endl;

  // --- Verify the Motion ---
  double tracking_norm = VoUtil.normofTransform(Rodrigues_R, temp_T);
  if(inliers.rows < VoUtil.track_min_inlier)
  {
      cout << "Too few inliners, No. of inliner is: " << inliers.rows << endl << endl;

      if(tracking_norm>= VoUtil.KF_update_norm)
        VoUtil.looseTrackingCount = VoUtil.looseTrackingRestart;
      else
        VoUtil.looseTrackingCount++;
      delete currKF_L;
      delete currKF_R;
      return false; // // finish this call-back func
  }

  if(VoUtil.DEBUG_MODE)
    cout << endl << "[DEBUG] The tracking norm is: " << tracking_norm << endl << endl;
  if(tracking_norm > VoUtil.track_max_norm)
  {
      cout << "Norm of Transform is too large !!" << endl;
      delete currKF_L;
      delete currKF_R;
      VoUtil.looseTrackingCount++;
      return false; // // finish this call-back func
  }


  //## Second solvePnP
  /*
  // Create new kp_3 and kp_2 vector
  vector<Point3f> track_kp_3f_new;  // Tracking matched 3d keypoints
  vector<Point2f> track_kp_2f_new;  // Tracking matched 2d keypoints
  for(int i=0; i<inliers.rows; i++)
  {
     track_kp_2f_new.push_back(currKF_L->track_kp_2f[inliers.ptr<int>(i)[0]]);
     track_kp_3f_new.push_back(VoUtil.prevKF_L.track_kp_3f[inliers.ptr<int>(i)[0]]);
  }
  solvePnP(track_kp_3f_new, track_kp_2f_new, CamPara.projMatrix, Mat(), Rodrigues_R, temp_T, true);
  */


  // Convert to Eigen
  Rodrigues( Rodrigues_R, temp_R );
  Map<Matrix<double, Dynamic, Dynamic, RowMajor>> Eigen_R(temp_R.ptr<double>(), 3, 3);
  Map<Vector3d> Eigen_T(temp_T.ptr<double>(), 3, 1);

  // Update Odometry data
  Matrix3d tempEigenR = Eigen_R;
  voTrack.odom_q = tempEigenR.transpose();
  voTrack.odom_q.normalize();
  voTrack.odom_t = -voTrack.odom_q._transformVector(Eigen_T);

  voTrack.keyframe_q.normalize();
  voTrack.globalpose_q = voTrack.keyframe_q * voTrack.odom_q;
  voTrack.globalpose_q.normalize();
  voTrack.globalpose_t = voTrack.keyframe_q._transformVector(voTrack.odom_t) + voTrack.keyframe_t;

  /*voTrack.globalpose_q = voTrack.odom_q*voTrack.keyframe_q;
  voTrack.globalpose_q.normalize();
  voTrack.globalpose_t = voTrack.odom_q._transformVector(voTrack.keyframe_t) + voTrack.odom_t;
  */
  // Update timestamp
  voTrack.timeStamp_prev = voTrack.timeStamp_curr;
  voTrack.timeStamp_curr = ImgMsgTime;

  if(VoUtil.DEBUG_MODE)
  {
    cout << "odom_q: " << voTrack.odom_q.w() << ", " << voTrack.odom_q.x() << ", " << voTrack.odom_q.y() << ", " << voTrack.odom_q.z() << endl;
    cout << "odom_t: " << voTrack.odom_t.transpose() << endl;

    //VoUtil.prevKF_L.KFLocal_A  for keyframe base
    voTrack.Global_Track_A.linear()      = VoUtil.prevKF_L.KFLocal_A.rotation()*( Eigen_R.transpose() );
    voTrack.Global_Track_A.translation() = (-voTrack.Global_Track_A.rotation()*Eigen_T) + VoUtil.prevKF_L.KFLocal_A.translation();
  }

  // --- Count the computation time ---
  if(VoUtil.SHOW_INFO)
  {
    auto t1_temp = std::chrono::high_resolution_clock::now(); // Set toc time
    cout << "Processing Time: " <<  1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t1_temp-t0).count() << endl;
    cout << "Processing Hz: " <<  1.0/(1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t1_temp-t0).count()) << endl;
  }

  // ====================
  // --- Draw Results ---
  // ====================
  if(VoUtil.SHOW_WINS)
    drawResult(currKF_R, currKF_L, matches, goodMatches, inliers, voFrame, VoUtil, CamPara);

  // --- Show the interested info ---
  if(VoUtil.SHOW_INFO)
  {
      cout << "Tracking good matches: " << goodMatches.size() << endl;
      cout << "PNP inliers: "<< inliers.rows << endl;
      cout << "The tracking norm is: " << tracking_norm << endl << endl;
      cout << "temp T ="  << endl << temp_T << endl;
      cout << "temp R ="  << endl << temp_R << endl;
      cout << "Affine R   ="  << endl << voTrack.globalpose_q.toRotationMatrix() << endl;
      cout << "Affine T   ="  << endl << voTrack.globalpose_t.transpose() << endl << endl;
  }

  // =======================
  // --- Update KeyFrame ---
  // =======================
  if(tracking_norm >= VoUtil.KF_update_norm || inliers.rows <= VoUtil.KF_min_inlier || goodMatches.size() <= VoUtil.KF_min_goodMatches)
  {
      //VoUtil.KeyFrame_L.push_back(*currKF_L);
      VoUtil.prevKF_L = *currKF_L; // Store current KeyFrame object into "prevKF_L" object
      //this->prevKF_L.KFLocal_R = this->Global_Track_R;
      //this->prevKF_L.KFLocal_T = this->Global_Track_T;
      if(VoUtil.DEBUG_MODE)
        VoUtil.prevKF_L.KFLocal_A = voTrack.Global_Track_A;
      voTrack.keyframe_q = voTrack.globalpose_q;
      voTrack.keyframe_t = voTrack.globalpose_t;
      voTrack.timeStamp_keyframe = voTrack.timeStamp_curr;

      voFrame.prev_grey_L = voFrame.curr_grey_L.clone(); // Store current image into "prev_grey_L" matrix
      voFrame.prev_grey_R = voFrame.curr_grey_R.clone(); // Store current image into "prev_grey_R" matrix
      voFrame.prev_bgr_L = voFrame.curr_bgr_L.clone();
      voFrame.prev_bgr_R = voFrame.curr_bgr_R.clone();
      if(VoUtil.SHOW_INFO)
        cout << "Keyframe Update !!";
  }

  delete currKF_L;
  delete currKF_R;
  VoUtil.looseTrackingCount = 0;
  return true;
}

#endif // VO_CORE
