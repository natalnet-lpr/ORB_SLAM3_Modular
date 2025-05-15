#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    //Create SLAM system. This is also the visualizer.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    while(true); //Necessary to keep the program running

    //Stop all threads
    SLAM.Shutdown();

    return 0;
}
