#include <iostream>
#include <vector>
#include <thread>
#include <memory>
#include <cstdlib> // for rand()
#include "Map.h"
#include "MapPoint.h"
#include "Viewer.h"
#include "System.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

using namespace std;
using namespace ORB_SLAM3;

int main(int argc, char **argv)
{
    /*
    // Create an empty ORB Vocabulary (Viewer expects it)
    ORBVocabulary* pVoc = new ORBVocabulary();

    // Create empty KeyFrame Database and Map
    KeyFrameDatabase* pKFDB = new KeyFrameDatabase(*pVoc);
    Map* pMap = new Map();

    // Create some fake MapPoints
    for(int i = 0; i < 50; ++i)
    {
        // Create a random 3D point
        float x = static_cast<float>(rand() % 100) / 10.0f;
        float y = static_cast<float>(rand() % 100) / 10.0f;
        float z = static_cast<float>(rand() % 100) / 10.0f;

        cv::Mat pos = (cv::Mat_<float>(3,1) << x, y, z);
        MapPoint* pMP = new MapPoint(pos, nullptr, pMap);

        // Insert into the Map
        pMap->AddMapPoint(pMP);
    }

    // Create the Viewer
    Viewer* pViewer = new Viewer(nullptr, pMap);

    // Launch Viewer in a separate thread
    thread thViewer(&Viewer::Run, pViewer);

    // Simulate main program running
    cout << "Viewer launched. Press [q] in the viewer window to quit." << endl;
    while (!pViewer->isFinished()) {
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    // Cleanup
    thViewer.join();
    delete pViewer;
    delete pMap;
    delete pKFDB;
    delete pVoc;
    */

    return 0;
}