#include <iostream>
#include <chrono>
#include "DBoW2/DBoW2.h"

using namespace DBoW2;
using namespace std;

int main()
{
  OrbVocabulary voc;
  bool loaded = true;

  cout << "Loading vocabulary from file..." << endl;

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  //voc.load("ORBvoc.txt.tar.gz"); // there are no yml.tar.gz files to be loaded
  loaded = voc.loadFromTextFile("ORBvoc.txt"); // too slow (MacOS)
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

  if(loaded)
  {
    double time = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout << "Vocabulary loaded (" << time << " seconds)." << endl;
  }
  else{
    cout << "Problem loading the vocabulary file.\n";
  }

  return 0;
}


