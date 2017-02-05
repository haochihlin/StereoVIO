#ifndef KEYFRAME
#define KEYFRAME

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace cv;

class KeyFrame
{
    public:
        vector<KeyPoint> ori_kp;
        vector<Point2f> ori_pt;
        Mat ori_desc, desc; // Keypoints Descriptor
        vector<Point3f> kp_3f;  // Disparity matched 3d keypoints
        vector<Point2f> kp_2f;  // Disparity matched 2d keypoints
        vector<Point3f> track_kp_3f;  // Tracking matched 3d keypoints
        vector<Point2f> track_kp_2f;  // Tracking matched 2d keypoints

        Mat KFLocal_T, KFLocal_R;
        Eigen::Affine3d KFLocal_A;

        void ClearOriKP()
        {
            ori_kp.clear();
            ori_pt.clear();
            ori_desc.release();
        }

        void ClearTrackKP()
        {
            track_kp_3f.clear();
            track_kp_2f.clear();
        }

};

#endif // KEYFRAME
