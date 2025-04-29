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

int main(int argc, char **argv)
{
    string vocabularyFile, settingsFile, imageFile;

    if(argc != 4)
    {
        cerr << "Usage: ./test_frame path_to_vocabulary path_to_settings path_to_image" << endl;
        return 1;
    }

    vocabularyFile = argv[1];
    settingsFile = argv[2];
    imageFile = argv[3];

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

    //Load image
    cv::Mat img = cv::imread(imageFile, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        cerr << "ERROR: Could not load image at: " << imageFile << endl;
        return -1;
    }

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

    //Timestamp
    double ts = 0.0;
    //Baseline
    double bf = 1.0;
    //Depth thr.
    double thDepth = 1.0;

    //Create a Frame:
    //gray, timestamp, extractor, voc, GeometricCamera, dist, bf, depth
    Frame frame = Frame(img, ts, extractor, vocabulary,
                        camModel, distCoef, bf, thDepth);

    //Display some Frame info
    cout << "Frame ID: " << frame.mnId << endl;
    cout << "Number of Keypoints: " << frame.N << endl;
    cout << "First KeyPoint: " << frame.mvKeys[0].pt << endl;

    return 0;
}
