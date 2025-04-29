#include <iostream>
#include "KeyFrameDatabase.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h"

using namespace std;
using namespace ORB_SLAM3;

int main() {
    // Assume you already have a vocabulary loaded
    ORBVocabulary* pVocabulary = new ORBVocabulary();
    
    // Normally you would load it from a file:
    // pVocabulary->loadFromTextFile("ORBvoc.txt");
    // Here we assume it's already loaded for simplicity.

    // Create KeyFrameDatabase
    KeyFrameDatabase* pKFDB = new KeyFrameDatabase(*pVocabulary);

    // Create a dummy KeyFrame
    // (In real usage, this would be a real KeyFrame extracted from images)
    Frame dummyFrame;   // you would have to construct this properly
    Map* dummyMap = nullptr;  // no real map for this demo
    KeyFrame* pKF = new KeyFrame(dummyFrame, dummyMap, pKFDB);

    // Add the KeyFrame to the database
    pKFDB->add(pKF);
    cout << "KeyFrame added to database." << endl;

    // Now perform a dummy query (normally for loop closure or relocalization)
    DBoW2::BowVector bowVec = pKF->mBowVec; // The Bag of Words vector of the keyframe
    vector<KeyFrame*> vpCandidates = pKFDB->DetectRelocalizationCandidates(&dummyFrame, dummyMap);

    // Show results
    cout << "Found " << vpCandidates.size() << " relocalization candidates." << endl;

    // Clean up
    delete pKFDB;
    delete pVocabulary;
    delete pKF;

    return 0;
}