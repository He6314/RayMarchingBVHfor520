//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;

#include "BHV.h"

#include <vector>

using namespace std;

//class BVHBone
vector<BVHBone>  BVHBone::Children() {
	return this->children;
}
void BVHBone::setChildren(vector<BVHBone> value) {
	this->children = value;
}

bool BVHBone::isRoot() {
	return this->root;
}
void BVHBone::setRoot(bool value) {
	this->root = value;
}

bool BVHBone::isEnd() {
	return this->end;
}
void BVHBone::setEnd(bool value) {
	this->end = value;
}

TransAxis BVHBone::Axis() {
	return this->axis;
}
void BVHBone::Axis(TransAxis value) {
	this->axis = value;
}

int BVHBone::MotionSpace() {
	return this->motionSpace;
}
void BVHBone::MotionSpace(int value) {
	this->motionSpace = value;
}

int BVHBone::Depth() {
	return this->depth;
}

int BVHBone::ChannelCount() {
	//通道数（enum类BVHChannel数组的大小）
	return this->channels.size();
}

string BVHBone::Name() {
	return this->name;
}

BVHBone* BVHBone::Parent()
{
	return this->parent;
}

vector<BVHChannel> BVHBone::Channels() {
	return this->channels;
}
void  BVHBone::Channels(vector<BVHChannel> value) {
	this->channels = value;
}

BVHBone::BVHBone()
{
	this->parent = NULL;
	this->index = 0; //节点的序号？
	this->name = "null";    //节点名称

	this->axis = TransAxis::None;    //运动类型
	if (this->parent != NULL)
		this->depth = this->parent->Depth() + 1;//节点级数（深度）
	else
	{
		this->depth = 0;
		this->root = true;//根节点设为0级
	}
	//channels = new BVHChannel[nrChannels];//建立若干通道（3或6）nrChannels为通道数
	int ind = 5;
	for (int k = 6 - 1; k >= 0; k--)
	{
		//channels[k] = (BVHChannel)ind;
		this->channels.insert(this->channels.begin(), BVHChannel::_BVHChannel(ind));
		ind--;
	}//为所有通道依次赋值
}

BVHBone::BVHBone(BVHBone* Parent, string Name, int nrChannels, TransAxis Axis)
{
	this->parent = Parent;//父节点
	this->index += index; //节点的序号？
	this->name = Name;    //节点名称

	this->axis = Axis;    //运动类型
	if (this->parent != NULL)
		this->depth = this->parent->Depth() + 1;//节点级数（深度）
	else
	{
		this->depth = 0;
		this->root = true;//根节点设为0级
	}
	//channels = new BVHChannel[nrChannels];//建立若干通道（3或6）nrChannels为通道数
	int ind = 5;
	for (int k = nrChannels - 1; k >= 0; k--)
	{
		//channels[k] = (BVHChannel)ind;
		this->channels.insert(this->channels.begin(), BVHChannel::_BVHChannel(ind));
		ind--;
	}//为所有通道依次赋值
}

void BVHBone::setTransOffset(double xOff, double yOff, double zOff)
{
	this->translOffset = new double[3]{ xOff, yOff, zOff };
}

void BVHBone::setRotOffset(double xOff, double yOff, double zOff)
{
	this->rotOffset = new double[3]{ xOff, yOff, zOff };
}

bool BVHBone::operator==(const BVHBone &right)const {
	return (this->name == right.name);
}


//class BVHSkeleton
vector<BVHBone> BVHSkeleton::Bones() {
	return this->bones;
}

int BVHSkeleton::Channels() {
	return this->channels;
}
void BVHSkeleton::Channels(int value) {
	this->channels = value;
}

int BVHSkeleton::MaxDepth() {
	return this->maxDepth;
}
void BVHSkeleton::MaxDepth(int value) {
	this->maxDepth = value;
}

int BVHSkeleton::NrBones() {
	return this->nrBones;
}
//void BVHSkeleton()
//{
//	bones = new List<BVHBone>();
//}

void BVHSkeleton::AddBone(BVHBone Bone)
{
	if (find(this->bones.begin(), this->bones.end(), Bone) == this->bones.end())
	{
		this->bones.push_back(Bone);
	}
}

void BVHSkeleton::FinalizeBVHSkeleton()//完成父子关系，建立End节点、计算总通道数
{
	this->channels = 0;
	for (unsigned int k = 0; k < this->bones.size(); k++)   //遍历所有节点
	{
		// set max Depth 查看最大的深度：依次比较每一骨骼点的深度
		if (this->bones[k].Depth() > this->maxDepth)
			this->maxDepth = this->bones[k].Depth();

		//set Bone Index for Motion Values Array 计算运动通道数（每个骨骼点的通道数*骨骼点数量）
		int motionCount = 0;
		for (unsigned int n = 0; n < k; n++)
		{
			motionCount += this->bones[n].ChannelCount();
		}
		this->bones[k].MotionSpace(motionCount);  //计算当前骨骼点运动数据开始的位置

											 //set Count of Channels for Skeleton 计算骨架总的通道数
		this->channels += this->bones[k].ChannelCount();

		//set Children 建立一个“childrenList”，并检索骨架内所有以本点为父节点的骨骼点。如没有则为end，如有，把childrenList的值赋给本点的children变量
		vector<BVHBone> childBoneList;
		for (unsigned int i = 0; i < this->bones.size(); i++)
		{
			if (this->bones[i].Parent() == &this->bones[k])
			{
				childBoneList.push_back(bones[i]);
			}
		}

		if (childBoneList.size() == 0)
		{
			this->bones[k].setEnd(true);
		}
		else
		{
			this->bones[k].setChildren(childBoneList);
		}
	}
	this->nrBones = this->bones.size();
}

void BVHSkeleton::copyParameters(BVHSkeleton input)
{
	this->channels = input.Channels();
	this->maxDepth = input.MaxDepth();
	this->nrBones = input.NrBones();
}

BVHBone* BVHSkeleton::findBoneByName(string targetName)
{
	for (unsigned int i = 0; i < this->bones.size(); i++)
	{
		BVHBone curBone = this->bones[i];
		if (curBone.Name() == targetName)
			return &this->bones[i];
	}
	return NULL;
}



