#ifndef CAMERAPARA
#define CAMERAPARA
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class CameraPara
{
    public:
        // Variable declaration
        int width;
        int height;
        double baseline; // meter
        double focal_length;
        double f_times_B; //f*Baseline
        double c_x;
        double c_y;

        Mat rmap[2][2]; // for caliberation remapping
        Mat cameraMatrix_L, distCoeffs_L, R_1, projMatrix_L, Cam2Imu_L;
        Mat cameraMatrix_R, distCoeffs_R, R_2, projMatrix_R, projMatrix, Cam2Imu_R;
        Size imgSize_L, imgSize_R;

        void SetValues(int init_width, int init_height, double init_baseline, double init_focal_length,
                   double init_f_times_B, double init_c_x, double init_c_y)
        {
            width = init_width;
            height = init_height;
            baseline = init_baseline;
            focal_length = init_focal_length;
            f_times_B = init_f_times_B;
            c_x = init_c_x;
            c_y = init_c_y;
        }

        bool LoadCalibration(string right_path, bool debugInfo = 0)
        {
            FileStorage fs_R(right_path, FileStorage::READ);
            if(!fs_R.isOpened())
                return false;
            fs_R["projection_matrix"] >> projMatrix_R;
            fs_R["image_width"] >> width;
            fs_R["image_height"] >> height;
            fs_R["cam2imu_matix"] >> Cam2Imu_R;
            if(debugInfo)
              cout << "Right projection matrix: " << endl << projMatrix_R << endl;
            fs_R.release();

            // Update param
            baseline = abs( projMatrix_R.at<double>(0, 3)/projMatrix_R.at<double>(0, 0) );
            focal_length = projMatrix_R.at<double>(0, 0);
            f_times_B = abs( projMatrix_R.at<double>(0, 3));
            c_x = abs( projMatrix_R.at<double>(0, 2));
            c_y = abs( projMatrix_R.at<double>(1, 2));
            imgSize_L = Size(width,height);
            imgSize_R = imgSize_L;

            projMatrix = projMatrix_R(Range::all(),Range(0, 3));

            return true;
        }

        bool LoadCalibration(const string left_path, const string right_path, bool debugInfo = 0)
        {
            int size_w, size_h;

            FileStorage fs_L(left_path, FileStorage::READ);
            if(!fs_L.isOpened())
                return false;
            fs_L["camera_matrix"] >> cameraMatrix_L;
            fs_L["distortion_coefficients"] >> distCoeffs_L;
            fs_L["rectification_matrix"] >> R_1;
            fs_L["projection_matrix"] >> projMatrix_L;
            fs_L["image_width"] >> size_w;
            fs_L["image_height"] >> size_h;
            fs_L["cam2imu_matix"] >> Cam2Imu_L;
            imgSize_L = Size(size_w,size_h);
            if(debugInfo)
            {
              cout << "Left camera matrix: " << endl << cameraMatrix_L << endl
                   << "Left distortion coeffs: " << endl << distCoeffs_L << endl
                   << "Left rectification matrix: " << endl << R_1 << endl
                   << "Left projection matrix: " << endl << projMatrix_L << endl;
            }
            fs_L.release();

            FileStorage fs_R(right_path, FileStorage::READ);
            if(!fs_R.isOpened())
                return false;
            fs_R["camera_matrix"] >> cameraMatrix_R;
            fs_R["distortion_coefficients"] >> distCoeffs_R;
            fs_R["rectification_matrix"] >> R_2;
            fs_R["projection_matrix"] >> projMatrix_R;
            fs_R["image_width"] >> size_w;
            fs_R["image_height"] >> size_h;
            fs_R["cam2imu_matix"] >> Cam2Imu_R;
            imgSize_R = Size(size_w,size_h);
            if(debugInfo)
            {
              cout << "Right camera matrix: " << endl << cameraMatrix_R << endl
                   << "Right distortion coeffs: " << endl << distCoeffs_R << endl
                   << "Right rectification matrix: " << endl << R_2 << endl
                   << "Right projection matrix: " << endl << projMatrix_R << endl;
            }
            fs_R.release();

            // Update param
            width = size_w;
            height = size_h;
            baseline = abs( projMatrix_R.at<double>(0, 3)/projMatrix_R.at<double>(0, 0) );
            focal_length = projMatrix_R.at<double>(0, 0);
            f_times_B = abs( projMatrix_R.at<double>(0, 3));
            c_x = abs( projMatrix_R.at<double>(0, 2));
            c_y = abs( projMatrix_R.at<double>(1, 2));

            if(imgSize_L.area() != imgSize_R.area())
            {
              cout << "[ERROR] Size of two images are not the same" << endl;
              return false;
            }

            projMatrix = projMatrix_R(Range::all(),Range(0, 3));
            initUndistortRectifyMap(cameraMatrix_L, distCoeffs_L, R_1, projMatrix_L, imgSize_L, CV_16SC2, rmap[0][0], rmap[0][1]);
            initUndistortRectifyMap(cameraMatrix_R, distCoeffs_R, R_2, projMatrix_R, imgSize_R, CV_16SC2, rmap[1][0], rmap[1][1]);

            return true;
        }

};

#endif // CAMERAPARA
