#include <iostream>
#include "vo_core/stereo_vo.h"

int main(int argc, char** argv) 
{
    ros::init( argc, argv, "Stereo_VO" );
    cout << endl << "===============================" << endl;
    cout         << "========== Stereo VO ==========" << endl;
    cout         << "===============================" << endl << endl;
    StereoVO stereoVO;
    ros::AsyncSpinner spinner(stereoVO.CPUMultiThreadNo); // Use multi threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
