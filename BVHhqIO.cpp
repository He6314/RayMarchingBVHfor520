//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;
//using System.IO;
//using System.Windows.Forms;

#include "BVHhqIO.h"

#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include "BHV.h"

using namespace std;

BVHSkeleton ReadingBVH::readSkeleton(string filename)
{
	BVHSkeleton bvhSkeleton;

	string strLine;

	BVHBone boneToAdd;

	TransAxis JointType;
	int depth = 0;
	double* offSets = new double[3];
	int numChannels = 0;
	int noEndset = 0;

	ifstream aFile(filename);
	if (!aFile.is_open())
		printf("File doesn't exist£¡\n");

	char charLine[10000];

	while (aFile.getline(charLine, 10000))
	{
		string strLine = string(charLine);

		//Conform the level of the bone
		if (strLine.find("{") != -1)
			depth++;
		if (strLine.find("}") != -1)
		{
			depth--;
			names.pop_back();
		}
		//Record the name and type of bones
		if ((strLine.find("ROOT") != -1) || (strLine.find("JOINT") != -1))
		{
			int indexOfSpace = strLine.find(' ');
			this->names.insert(this->names.end(), strLine.substr(indexOfSpace + 1));
		}
		if (strLine.find("End") != -1)
		{
			noEndset++;
			this->names.insert(this->names.end(), "EndSet_" + to_string(noEndset));
		}

		//Record the type and number of offset
		if (strLine.find("OFFSET") != -1)
		{
			
			string numberText = strLine.substr(strLine.find("T") + 2);
			
			for (int i = 0; i < 3; i++)
			{
				if (numberText.find('\t') != -1) {
					string strNumber = (numberText.substr(0, numberText.find('\t')));
					offSets[i] = atof(strNumber.c_str());
					numberText = numberText.substr(numberText.find('\t') + 1);
				}
				else {
					string strNumber = (numberText.substr(0, numberText.find(' ')));
					offSets[i] = atof(strNumber.c_str());
					numberText = numberText.substr(numberText.find(' ') + 1);
				}
			}

			if (offSets[0] == 0 && offSets[1] == 0 && offSets[2] > 0)
				JointType = TransAxis::_TransAxis(3);//??? Z
			else if (offSets[0] == 0 && offSets[1] == 0 && offSets[2] < 0)
				JointType = TransAxis::_TransAxis(6); //nZ;
			else if (offSets[0] == 0 && offSets[1] > 0 && offSets[2] == 0)
				JointType = TransAxis::_TransAxis(2); //Y;
			else if (offSets[0] == 0 && offSets[1] < 0 && offSets[2] == 0)
				JointType = TransAxis::_TransAxis(5); //nY;
			else if (offSets[0] > 0 && offSets[1] == 0 && offSets[2] == 0)
				JointType = TransAxis::_TransAxis(1); //X;
			else if (offSets[0] < 0 && offSets[1] == 0 && offSets[2] == 0)
				JointType = TransAxis::_TransAxis(4); //nX;
			else
				JointType = TransAxis::_TransAxis(0); //None;


													  //Conform the Endsite
			if (!names[depth - 1].empty() && (names[depth - 1].find("End") != -1))
			{
				BVHBone* tmpParent = bvhSkeleton.findBoneByName(names[depth - 2]);
				BVHBone tmpBoneToAdd(tmpParent, names[depth - 1], 0, JointType);
				tmpBoneToAdd.setRoot(false);
				tmpBoneToAdd.setEnd(true);
				tmpBoneToAdd.setTransOffset(offSets[0], offSets[1], offSets[2]);
				boneToAdd = tmpBoneToAdd;
				bvhSkeleton.AddBone(boneToAdd);
			}
		}
		//Record the number of channels and finish the recording of this bone
		if (strLine.find("CHANNELS") != -1)
		{
			string temp = strLine.substr(strLine.find("S") + 2, 1);
			numChannels = atoi(temp.c_str());

			if (depth - 1 == 0)
			{
				BVHBone tmpBoneToAdd(NULL, names[depth - 1], numChannels, JointType);
				tmpBoneToAdd.setRoot(true);
				tmpBoneToAdd.setEnd(false);
				if (strLine.find("Zr") < strLine.find("Xr"))
				{
					vector<BVHChannel> tempChn;
					tempChn.push_back(tmpBoneToAdd.Channels()[0]);
					tempChn.push_back(tmpBoneToAdd.Channels()[1]);
					tempChn.push_back(tmpBoneToAdd.Channels()[2]);
					tempChn.push_back(tmpBoneToAdd.Channels()[5]);
					tempChn.push_back(tmpBoneToAdd.Channels()[3]);
					tempChn.push_back(tmpBoneToAdd.Channels()[4]);
					tmpBoneToAdd.Channels(tempChn);
				}
				boneToAdd = tmpBoneToAdd;
			}

			else
			{
				BVHBone* tmpParent = bvhSkeleton.findBoneByName(names[depth - 2]);
				BVHBone tmpBoneToAdd(tmpParent, names[depth - 1], numChannels, JointType);
				tmpBoneToAdd.setRoot(false);
				if (strLine.find("Z") < strLine.find("X"))
				{
					BVHChannel tempChn;
					tempChn = tmpBoneToAdd.Channels()[2];
					tmpBoneToAdd.Channels()[2] = tmpBoneToAdd.Channels()[1];
					tmpBoneToAdd.Channels()[1] = tmpBoneToAdd.Channels()[0];
					tmpBoneToAdd.Channels()[0] = tempChn;
				}
				boneToAdd = tmpBoneToAdd;
			}

			boneToAdd.setTransOffset(offSets[0], offSets[1], offSets[2]);
			bvhSkeleton.AddBone(boneToAdd);
		}

		//strLine = sr.ReadLine();
	}

	bvhSkeleton.FinalizeBVHSkeleton();

	this->numSkeChannel = bvhSkeleton.Channels();
	this->maxDepth = bvhSkeleton.MaxDepth();

	if (!bvhSkeleton.Bones().empty() && (numChannels > 0) && (this->maxDepth > 0))
	{
		printf("Skeleton successfully loaded£º\n");
		//for (unsigned int i = 0; i < bvhSkeleton.Bones().size(); i++)
		//{
		//	for (int j = 0; j < bvhSkeleton.Bones()[i].Depth(); j++)
		//	{
		//		cout << "\t";
		//	}
		//	cout << bvhSkeleton.Bones()[i].Name() << endl;
		//}
		this->skeletonReady = true;
	}
	else
		printf("Loadin failed! Skeleton\n");

	aFile.close();

	return bvhSkeleton;
}

double** ReadingBVH::readData(string filename)
{
	double** data = new double*[20];
	for (int i = 0; i < 20; i++)
		data[i] = new double[20];

	string strLine;
	int frameNow = 0;
	//try
	//{
	ifstream aFile(filename);
	char charLine[100000];

	while (aFile.getline(charLine, 100000))
	{
		string strLine = string(charLine);

		if (strLine.find("Frames") != -1)
		{
			string frameStr = strLine.substr(strLine.find("s") + 3);
			this->nrFrames = atoi(frameStr.c_str());
			data = new double*[this->nrFrames];
			for (int i = 0; i < this->nrFrames; i++)
				data[i] = new double[this->numSkeChannel];
		}
		if (strLine.find("Frame Time") != -1)
		{
			string timeStr = strLine.substr(strLine.find(":") + 2);
			this->timePerFrame = atof(timeStr.c_str());
		}

		if (strLine.size() > 100)
		{
			string dataText = strLine;

			for (int j = 0; j < this->numSkeChannel; j++)
			{
				if (strLine.find('\t') != -1) {
					string strData = (dataText.substr(0, dataText.find('\t')));
					data[frameNow][j] = atof(strData.c_str());
					dataText = dataText.substr(dataText.find('\t') + 1);
				}
				else {
					string strData = (dataText.substr(0, dataText.find(' ')));
					data[frameNow][j] = atof(strData.c_str());
					dataText = dataText.substr(dataText.find(' ') + 1);
				}
			}
			frameNow++;
		}

		//strLine = sr.ReadLine();
	}

	if ((data != NULL) && (this->nrFrames > 0) && (this->numSkeChannel > 0) && (this->timePerFrame > 0))
	{
		this->MotionData = data;
		this->motionReady = true;
		printf("Motion data successfully loaded£º\n");
		//for (unsigned int i = 0; i < this->nrFrames; i++)
		//{
		//	for (int j = 0; j < this->numSkeChannel; j++)
		//	{
		//		cout<<this->MotionData[i][j]<<"\t";
		//	}
		//	cout<<endl;
		//}
	}
	else
		printf("Load failed! Motion data");

	//sr.Close();
//}
//catch (IOException ex)
//{
//	MessageBox.Show("¶ÁÈ¡´íÎó£º");
//	MessageBox.Show(ex.ToString());
//	return data;
//}
	return data;
}

ReadingBVH::ReadingBVH()
{
	this->readStart = false;
	this->readComplete = false;
	//names = new string[15];
	this->nrFrames = 0;
	this->timePerFrame = 0;
	this->numSkeChannel = 0;
	this->maxDepth = 0;
}

bool ReadingBVH::ReadStart() {
	return this->readStart;
}

bool ReadingBVH::ReadComplete() {
	return this->readComplete;
}

int ReadingBVH::NumOfFrames() {
	return this->nrFrames;
}

double ReadingBVH::TimePerFrame() {
	return this->timePerFrame;
}

int ReadingBVH::NumSkeChannel() {
	return this->numSkeChannel;
}

int ReadingBVH::MaxDepth() {
	return this->maxDepth;
}

void ReadingBVH::ReadFromFile(string filename)
{
	this->readStart = true;
	this->skeletonReady = false;
	this->motionReady = false;

	string path = filename;

	this->Skeleton = readSkeleton(path);
	cout << this->Skeleton.NrBones() << endl;
	this->MotionData = readData(path);
	cout << this->nrFrames << ",	" << this->numSkeChannel << endl;

	if (this->skeletonReady && this->motionReady)
	{
		this->readComplete = true;
	}
	else
		printf("Load failed!\n");
};
