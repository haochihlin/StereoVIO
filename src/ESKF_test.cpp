#include <iostream>
#include "eskf_core/eskf_core.h"
#include <Eigen/Dense>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init( argc, argv, "ESKF_Prop" );
    cout << endl << "===============================" << endl;
    cout         << "========== ESKF_Prop ==========" << endl;
    cout         << "===============================" << endl << endl;
    ESKF_Core EskfCore;
    Eigen::Quaterniond cam2imu_q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d cam2imu_t = Eigen::Vector3d::Zero();
    EskfCore.StartProcessing(cam2imu_q, cam2imu_t);
    ros::AsyncSpinner spinner(2); // Use multi threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
