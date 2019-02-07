#pragma once

#include <vector>

using namespace std;

class BVHChannel
{
public:
	enum _BVHChannel
	{
		Xposition,
		Yposition,
		Zposition,
		Xrotation,
		Yrotation,
		Zrotation
	};
	BVHChannel() {
		enumValue = Xposition;
	}
	BVHChannel(_BVHChannel value) {
		enumValue = value;
	}
private:
	_BVHChannel enumValue;
};

class TransAxis
{
public:
	enum _TransAxis
	{
		None,
		X,
		Y,
		Z,
		nX,
		nY,
		nZ
	};
	TransAxis() {
		enumValue = None;
	}
	TransAxis(_TransAxis value) {
		enumValue = value;
	}
private:
	_TransAxis enumValue;
};

class BVHBone         // ···········骨骼点
{
private:
	BVHBone* parent;//父节点
	vector<BVHBone> children;//子节点（数组）

	string name;
	int depth;
	int index;
	vector<BVHChannel> channels;
	bool end;
	bool root;

	int motionSpace;
	TransAxis axis;

public:
	double* rotOffset = new double[3]{ 0, 0, 0 };
	double* translOffset = new double[3]{ 0, 0, 0 };

	vector<BVHBone> Children();
	void setChildren(vector<BVHBone> value);
	bool isRoot();
	void setRoot(bool value);
	bool isEnd();
	void setEnd(bool value);
	TransAxis Axis();
	void Axis(TransAxis value);
	int MotionSpace();
	void MotionSpace(int value);
	int Depth();
	int ChannelCount();
	string Name();
	BVHBone* Parent();
	vector<BVHChannel> Channels();
	void Channels(vector<BVHChannel> value);

	BVHBone();
	BVHBone(BVHBone* Parent, string Name, int nrChannels, TransAxis Axis);

	void setTransOffset(double xOff, double yOff, double zOff);
	void setRotOffset(double xOff, double yOff, double zOff);

	bool operator==(const BVHBone &right)const;
};

class BVHSkeleton
{
private:
	vector<BVHBone> bones;
	int maxDepth = 0;
	int nrBones;
	int channels;

public:
	vector<BVHBone> Bones();
	int Channels();
	void Channels(int value);
	int MaxDepth();
	void MaxDepth(int value);
	int NrBones();

	void AddBone(BVHBone Bone);
	void FinalizeBVHSkeleton();
	void copyParameters(BVHSkeleton input);
	BVHBone* findBoneByName(string targetName);
};


