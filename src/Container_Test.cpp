#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "eskf_core/eskf_container.h"
#include "eskf_core/state.h"
#include "eskf_core/eskf_core.h"
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace Eigen;
using boost::shared_ptr;

int main(int argc, char** argv)
{
    ros::init( argc, argv, "Container_Test" );
    cout << endl << "====================================" << endl;
    cout         << "========== Container_Test ==========" << endl;
    cout         << "====================================" << endl << endl;

    ESKF_Container<ESKF_State> StateBuffer;
    ESKF_Container<ESKF_State>::iterator_T StateBuffer_iter;
    boost::shared_ptr<ESKF_State> testState(new ESKF_State);

    for(int i =0; i<100; i++)
    {
      boost::shared_ptr<ESKF_State> currentState(new ESKF_State);
      currentState->p = Vector3d(0,0,(double)i);
      currentState->timestamp = (double)i;
      StateBuffer.Insert(currentState);
    }

    cout << "Size of StateBuffer: " << StateBuffer.Size() << endl;

    StateBuffer_iter = StateBuffer.GetIteratorBegin();
    cout << "The first data (iter): " << StateBuffer_iter->second->timestamp << ", " << StateBuffer_iter->second->p.transpose() << endl;
    testState = StateBuffer.GetFirst();
    cout << "The first data (obj): " << testState->timestamp << ", " << testState->p.transpose() << endl;

    testState = StateBuffer.GetLast();
    cout << "The last data (obj): " << testState->timestamp << ", " << testState->p.transpose() << endl;

    testState = StateBuffer.GetClosest(32.2);
    cout << "The data closet to 32.2 (obj): " << testState->timestamp << ", " << testState->p.transpose() << endl;

    cout << "\nCleaning order than 20.2" << endl;
    double age = 20.2;
    StateBuffer.ClearOlderThan( age);
    cout << "Size of StateBuffer: " << StateBuffer.Size() << endl;
    testState = StateBuffer.GetFirst();
    cout << "The first data after cleaning order than 20.2: " << testState->timestamp << ", " << testState->p.transpose() << endl;

    ros::AsyncSpinner spinner(2); // Use multi threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
