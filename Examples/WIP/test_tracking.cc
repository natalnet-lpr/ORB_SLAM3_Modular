#include <iostream>
#include <opencv2/opencv.hpp>

#include "System.h"
#include "Frame.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Converter.h"

#include <include/CameraModels/Pinhole.h>

using namespace std;
using namespace ORB_SLAM3;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
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

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}

int main(int argc, char **argv)
{
    string vocabularyFile, settingsFile;
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;

    if(argc != 4)
    {
        cerr << "Usage: ./test_tracking path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    vocabularyFile = argv[1];
    settingsFile = argv[2];

    //Load settings
    cv::FileStorage fsSettings(settingsFile, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "ERROR: Could not open settings file at: " << settingsFile << endl;
        return -1;
    }

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    ORBVocabulary* vocabulary = new ORBVocabulary();
    bool load_vocab = vocabulary->loadFromTextFile(vocabularyFile);
    if(!load_vocab)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << vocabularyFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    ///Retrieve paths to images
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    //Configure ORB extractor
    int nFeatures = fsSettings["ORBextractor.nFeatures"];
    float scaleFactor = fsSettings["ORBextractor.scaleFactor"];
    int nLevels = fsSettings["ORBextractor.nLevels"];
    int iniThFAST = fsSettings["ORBextractor.iniThFAST"];
    int minThFAST = fsSettings["ORBextractor.minThFAST"];
    ORBextractor *extractor = new ORBextractor(nFeatures, scaleFactor, nLevels, iniThFAST, minThFAST);

    //Configure camera
    float fx, fy, cx, cy;
    fx = fsSettings["Camera1.fx"];
    fy = fsSettings["Camera1.fy"];
    cx = fsSettings["Camera1.cx"];
    cy = fsSettings["Camera1.cy"];
    vector<float> camCalib{fx, fy, cx, cy};
    Pinhole* camModel = new Pinhole(camCalib);
    cv::Mat distCoef = cv::Mat::zeros(1, 5, CV_64F);
    distCoef.at<double>(0,0) = fsSettings["Camera1.k1"];
    distCoef.at<double>(0,1) = fsSettings["Camera1.k2"];
    distCoef.at<double>(0,2) = fsSettings["Camera1.p1"];
    distCoef.at<double>(0,3) = fsSettings["Camera1.p2"];
    distCoef.at<double>(0,4) = fsSettings["Camera1.k3"];

    //Baseline
    double bf = 1.0;
    //Depth thr.
    double thDepth = 1.0;

    //Process image sequence
    for(size_t i=0; i<vstrImageFilenames.size(); i++)
    {
        cv::Mat img = cv::imread(vstrImageFilenames[i], cv::IMREAD_GRAYSCALE);
        if(img.empty())
        {
            cerr << "ERROR: Could not load image at: " << vstrImageFilenames[i] << endl;
            return -1;
        }

        //Get timestamp
        double ts = vTimestamps[i];

        //Create a Frame:
        Frame frame = Frame(img, ts, extractor, vocabulary, camModel, distCoef, bf, thDepth);

        //Show keypoints using openCV
        cv::Mat imgWithKeypoints;
        img.copyTo(imgWithKeypoints);
        cv::drawKeypoints(img, frame.mvKeys, imgWithKeypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        cv::imshow("Keypoints", imgWithKeypoints);
        cv::waitKey(0);

        //Display some info
        cout << "Frame ID: " << frame.mnId << endl;
        cout << "Number of Keypoints: " << frame.N << endl;
        cout << "Number of mappoints: " << frame.mvpMapPoints.size() << endl;
    }

    //Clean up
    delete vocabulary;
    delete extractor;
    delete camModel;

    return 0;
}
