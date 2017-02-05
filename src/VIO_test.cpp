#include <iostream>
#include "vio_implement/vio_core.h"

int main(int argc, char** argv) 
{
    ros::init( argc, argv, "Stereo_VIO" );
    cout << endl << "================================" << endl;
    cout         << "========== Stereo VIO ==========" << endl;
    cout         << "================================" << endl << endl;
    StereoVIO stereoVIO;
    ros::AsyncSpinner spinner(stereoVIO.CPUMultiThreadNo); // Use multi CPU threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
