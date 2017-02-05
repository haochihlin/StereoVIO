#ifndef STEREO_VO
#define STEREO_VO

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

#include "vo_core/vo_core.h"

using namespace std;
using namespace cv;
using namespace Eigen;

class StereoVO
{
    public:
        StereoVO() : it_(nh_), OriImg_L_sub_( it_, "/cam0/image_raw", 1 ), OriImg_R_sub_( it_, "/cam1/image_raw", 1 ), sync( MySyncPolicy( 10 ), OriImg_L_sub_, OriImg_R_sub_ )
        {
            sync.registerCallback( boost::bind( &StereoVO::callback, this, _1, _2 ) );

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
            cout << endl << "--- Calibration files found ! ---" << endl;

            // Initialize parameters
            cout << "Init affline Matrix" << endl;
            cout << voTrack.Global_Track_A.rotation() << endl;
            cout << voTrack.Global_Track_A.translation() << endl;
            cout << endl << "Wait for the image inputs ..." << endl;

        }

        ~StereoVO()
        {
            destroyWindow(VoUtil.FourImgWin);
            //destroyWindow(TrajectWin);

            cout << endl << "Length of KF vector: " << VoUtil.KeyFrame_L.size() << endl;
            VoUtil.KeyFrame_L.clear();
            // Check the release result
            if(VoUtil.KeyFrame_L.size() == 0)
                cout << endl << "Released KF vector !" << endl;
        }

        // function declaration
        void callback(const sensor_msgs::ImageConstPtr& OriImg_L_msg,const sensor_msgs::ImageConstPtr& OriImg_R_msg);

        // external available parameters
        int CPUMultiThreadNo = 1;

        VOFrame voFrame; // struct definition for opencv mat
        VOTRack voTrack; //                   for global tracking matrix

    private:
        VO_Util VoUtil;
        CameraPara CamPara;
        VO_Core VoCore;

        // --- ROS Related ---
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        // Stereo images sync
        typedef image_transport::SubscriberFilter ImageSubscriber;
        ImageSubscriber OriImg_L_sub_;
        ImageSubscriber OriImg_R_sub_;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer< MySyncPolicy > sync;


};


//ROS callback
void StereoVO::callback(const sensor_msgs::ImageConstPtr& OriImg_L_msg,const sensor_msgs::ImageConstPtr& OriImg_R_msg)
{
    ROS_INFO("----- New frame arrived ! -----");
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

    // Primary function of VO
    VoCore.stereoVOProcess(this->voFrame, this->voTrack, this->VoUtil, this->CamPara, ImgMsgTime);

    // --- TF Publish info ---
    VoCore.VOGlobalTFPub(this->voTrack, this->VoUtil);
}

#endif
