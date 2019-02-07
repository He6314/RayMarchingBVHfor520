#pragma once

#include <string>
#include <fstream>
#include <vector>
#include "BHV.h"
using namespace std;

class ReadingBVH
	// To read a BVH file, and to record its data and information for processing and writing. 
{
private:
	bool readStart;
	bool readComplete;
	bool skeletonReady;
	bool motionReady;

	vector<string> names;// of bones
						 //record the header information£º
	int nrFrames;
	double timePerFrame;
	int numSkeChannel;//the sum of all channels of the bones in skeleton
	int maxDepth;//the number of layers of the skeleton structure

	BVHSkeleton readSkeleton(string filename);
	double** readData(string filename);

public:
	BVHSkeleton Skeleton;
	double** MotionData;

	ReadingBVH();

	bool ReadStart();
	bool ReadComplete();

	int NumOfFrames();
	double TimePerFrame();
	int NumSkeChannel();
	int MaxDepth();

	void ReadFromFile(string filename);
};

