#ifndef VIO_CORE
#define VIO_CORE

#include <iostream>
#include <vector>

// OpenCV Libs
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS Libs
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

// Eigen
#include <Eigen/Dense>

// Local Libs
#include "vo_core/vo_core.h"
#include "eskf_core/eskf_core.h"

using namespace std;
using namespace cv;
using namespace Eigen;

class StereoVIO
{
    public:
        StereoVIO();
        ~StereoVIO();
        // function declaration
        void callback(const sensor_msgs::ImageConstPtr& OriImg_L_msg,const sensor_msgs::ImageConstPtr& OriImg_R_msg);
        // external available parameters
        int CPUMultiThreadNo = 1;
        VOFrame voFrame; // struct definition for opencv mat
        VOTRack voTrack; //                   for global tracking matrix

    private:
        // --------------------
        // --- ESKF related ---
        // --------------------
        ESKF_Core EskfCore;
        bool eskf_trig_flag = false;
        bool eskf_cam2imu_found = false;
        Quaterniond cam2imu_q;
        Vector3d cam2imu_t;

        // -------------------------
        // --- Stereo VO related ---
        // -------------------------
        VO_Util VoUtil;
        CameraPara CamPara;
        VO_Core VoCore;

        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        // For stereo images sync
        typedef image_transport::SubscriberFilter ImageSubscriber;
        ImageSubscriber OriImg_L_sub_;
        ImageSubscriber OriImg_R_sub_;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer< MySyncPolicy > sync;
};

StereoVIO::StereoVIO() : it_(nh_), OriImg_L_sub_( it_, "/cam0/image_raw", 1 ), OriImg_R_sub_( it_, "/cam1/image_raw", 1 ), sync( MySyncPolicy( 10 ), OriImg_L_sub_, OriImg_R_sub_ )
{
    sync.registerCallback( boost::bind( &StereoVIO::callback, this, _1, _2 ) );

    CPUMultiThreadNo = VoUtil.CPUMultiThreadNo;
    namedWindow(VoUtil.FourImgWin);

    // ----------------------
    // Read camera parameters
    // ----------------------
    if(VoUtil.rectImgInputsId) // for rect image inputs
      CamPara.LoadCalibration(VoUtil.cam_cali_path_R, 0);
    else
      if(!CamPara.LoadCalibration(VoUtil.cam_cali_path_L, VoUtil.cam_cali_path_R, 0))
      {
          cout << "[ERROR] Calibration files not found" << endl;
          return ;
      }

    // Passing the cam to imu extrinsics matrix to eskf core
    if( this->VoUtil.Cam2Imu_CVMat2Eigen(this->CamPara.Cam2Imu_L, cam2imu_q, cam2imu_t) )
    {
      cout << "\n--- Camera to Imu Extrinsics Matrix found ! ---" << endl;
      cout << "Cam to Imu q: " << cam2imu_q.w() << ", " << cam2imu_q.vec().transpose() << endl;
      cout << "Cam to Imu t: " << cam2imu_t.transpose() << endl;
      eskf_cam2imu_found = true;
    }
    else
      cout << "\nCamera to Imu Extrinsics matrix not found, can not trigger eskf core !!" << endl;

    cout << endl << "Wait for the image inputs ..." << endl;
}

StereoVIO::~StereoVIO()
{
    destroyWindow(VoUtil.FourImgWin);
    //destroyWindow(TrajectWin);

    cout << endl << "Length of KF vector: " << VoUtil.KeyFrame_L.size() << endl;
    VoUtil.KeyFrame_L.clear();
    // Check the release result
    if(VoUtil.KeyFrame_L.size() == 0)
        cout << endl << "Released KF vector !" << endl;
}

//ROS callback
void StereoVIO::callback(const sensor_msgs::ImageConstPtr& OriImg_L_msg,const sensor_msgs::ImageConstPtr& OriImg_R_msg)
{
  // -------------------------
  // --- Handle two images ---
  // -------------------------
  if(this->VoUtil.DEBUG_MODE)
    ROS_INFO("\n----- New frame arrived ! -----");
  double ImgMsgTime = (OriImg_R_msg->header.stamp.toSec() + OriImg_L_msg->header.stamp.toSec())/2.0;
  cv_bridge::CvImagePtr cv_ptr_L, cv_ptr_R;
  try
  {
      cv_ptr_L = cv_bridge::toCvCopy(OriImg_L_msg, sensor_msgs::image_encodings::MONO8);
      cv_ptr_R = cv_bridge::toCvCopy(OriImg_R_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  this->voFrame.curr_grey_L = cv_ptr_L->image;
  this->voFrame.curr_grey_R = cv_ptr_R->image;

  if(!this->VoUtil.rectImgInputsId) // For un-rect image
  {
    remap(this->voFrame.curr_grey_L, this->voFrame.curr_grey_L, this->CamPara.rmap[0][0], this->CamPara.rmap[0][1], INTER_LINEAR);
    remap(this->voFrame.curr_grey_R, this->voFrame.curr_grey_R, this->CamPara.rmap[1][0], this->CamPara.rmap[1][1], INTER_LINEAR);
  }

  // ------------------------
  // --- VO & ESKF Update ---
  // ------------------------
  if( VoCore.stereoVOProcess(this->voFrame, this->voTrack, this->VoUtil, this->CamPara, ImgMsgTime) )
  {
    // --- VO based TF Publish ---
    //VoCore.VOGlobalTFPub(this->voTrack, this->VoUtil);
    // --- Trigger the ESKF Core ---
    if(this->eskf_trig_flag ==  false && eskf_cam2imu_found == true)
    {
      this->EskfCore.StartProcessing(this->cam2imu_q, this->cam2imu_t);
      this->eskf_trig_flag = true;
    }
    // --- Call ESKF Update ---
    if(this->EskfCore.MultiThreadUpdate(this->voTrack) == false)
      cout << "\nThe thread is now executing, waiting for it finished !" << endl;
  }
}

#endif // VIO_CORE
