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
	//ͨ������enum��BVHChannel����Ĵ�С��
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
	this->index = 0; //�ڵ����ţ�
	this->name = "null";    //�ڵ�����

	this->axis = TransAxis::None;    //�˶�����
	if (this->parent != NULL)
		this->depth = this->parent->Depth() + 1;//�ڵ㼶������ȣ�
	else
	{
		this->depth = 0;
		this->root = true;//���ڵ���Ϊ0��
	}
	//channels = new BVHChannel[nrChannels];//��������ͨ����3��6��nrChannelsΪͨ����
	int ind = 5;
	for (int k = 6 - 1; k >= 0; k--)
	{
		//channels[k] = (BVHChannel)ind;
		this->channels.insert(this->channels.begin(), BVHChannel::_BVHChannel(ind));
		ind--;
	}//Ϊ����ͨ�����θ�ֵ
}

BVHBone::BVHBone(BVHBone* Parent, string Name, int nrChannels, TransAxis Axis)
{
	this->parent = Parent;//���ڵ�
	this->index += index; //�ڵ����ţ�
	this->name = Name;    //�ڵ�����

	this->axis = Axis;    //�˶�����
	if (this->parent != NULL)
		this->depth = this->parent->Depth() + 1;//�ڵ㼶������ȣ�
	else
	{
		this->depth = 0;
		this->root = true;//���ڵ���Ϊ0��
	}
	//channels = new BVHChannel[nrChannels];//��������ͨ����3��6��nrChannelsΪͨ����
	int ind = 5;
	for (int k = nrChannels - 1; k >= 0; k--)
	{
		//channels[k] = (BVHChannel)ind;
		this->channels.insert(this->channels.begin(), BVHChannel::_BVHChannel(ind));
		ind--;
	}//Ϊ����ͨ�����θ�ֵ
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

void BVHSkeleton::FinalizeBVHSkeleton()//��ɸ��ӹ�ϵ������End�ڵ㡢������ͨ����
{
	this->channels = 0;
	for (unsigned int k = 0; k < this->bones.size(); k++)   //�������нڵ�
	{
		// set max Depth �鿴������ȣ����αȽ�ÿһ����������
		if (this->bones[k].Depth() > this->maxDepth)
			this->maxDepth = this->bones[k].Depth();

		//set Bone Index for Motion Values Array �����˶�ͨ������ÿ���������ͨ����*������������
		int motionCount = 0;
		for (unsigned int n = 0; n < k; n++)
		{
			motionCount += this->bones[n].ChannelCount();
		}
		this->bones[k].MotionSpace(motionCount);  //���㵱ǰ�������˶����ݿ�ʼ��λ��

											 //set Count of Channels for Skeleton ����Ǽ��ܵ�ͨ����
		this->channels += this->bones[k].ChannelCount();

		//set Children ����һ����childrenList�����������Ǽ��������Ա���Ϊ���ڵ�Ĺ����㡣��û����Ϊend�����У���childrenList��ֵ���������children����
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



