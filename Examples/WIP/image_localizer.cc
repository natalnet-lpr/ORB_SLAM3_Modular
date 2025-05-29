#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    cv::Mat im;

    if(argc != 4)
    {
        cerr << endl << "Usage: ./image_localizer path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    //Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    SLAM.ActivateLocalizationMode();
    SLAM.SetStepByStep(true);
    
    SLAM.PrintStatus();

    for(int ni=0; ni < vTimestamps.size(); ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

        cout << "Tracking frame " << ni << " (" << tframe << ")\n";
        SLAM.TrackMonocular(im, tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageFilenames[ni]);
    }

    //Stop all threads
    SLAM.Shutdown();

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    string strPrefixLeft = strPathToSequence + "/image_0/";

    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
