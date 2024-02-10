#pragma once
#include <vector>
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

struct BodyParam;
struct GeomParam;

struct GeomParam
{
	GeomParam() 
	{
		memset(this, 0, sizeof(GeomParam));
	};
	
	float fPos[3];
	float qOrient[4];
	float fRadius;
	float fHalfHeight;
	float fSize[3];

	char strType[64];

	GeomParam* pNext;
	GeomParam* pPrev;
};

struct JointParam
{
	JointParam()
	{
		memset(this, 0, sizeof(JointParam));
	}

	BodyParam* part1;
	BodyParam* part2;

	float axis[3];
	float range[2];
	float damping;
	float stiffness;
	float armature;

	JointParam* pNext;
	JointParam* pPrev;
};

struct BodyParam
{
	BodyParam()
	{
		memset(this, 0, sizeof(BodyParam));
	}

	float posLocal[3];
	float posGlobal[3];

	GeomParam* pGeoms;
	JointParam* pJoints;

	BodyParam* pChild;
	BodyParam* pSibling;
	BodyParam* pParent;

	void Destroy();
};

struct MujocoXmlParser
{
	MujocoXmlParser() {};

	BodyParam* Load(const char* pXmlFileName, float* posRoot = 0, float _scale = 1.0f);
	int ProcessBodyElement(
		BodyParam* _parentBody,
		std::vector<float> _parentPosVec,
		std::vector<float> _parentLocalPosVec,
		const XMLElement* _bodyElement, int depth);

	float fScale;
};