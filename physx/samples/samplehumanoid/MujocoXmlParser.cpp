#include <iostream>
#include <sstream>
#include <string>


#include "MujocoXmlParser.h"

void BodyParam::Destroy()
{
	BodyParam* pNow = this;
	while (0 != pNow)
	{
		BodyParam* _pNext = pNow->pSibling;
		// Delete the joints and geoms related to this Body.
		GeomParam* _pGeom = pNow->pGeoms;
		while (0 != _pGeom)
		{
			GeomParam* pNexGeom = _pGeom->pNext;
			// Delete the geom
			delete _pGeom;
			_pGeom = pNexGeom;
		} 

		JointParam* _pJoint = pNow->pJoints;
		while (0 != _pJoint)
		{
			JointParam* pNexJoint = _pJoint->pNext;
			// Delete the geom
			delete _pJoint;
			_pJoint = pNexJoint;
		}

		if (pNow->pChild)
		{
			pNow->pChild->Destroy();
		}

		delete pNow;
		pNow = _pNext;
	} 

}

std::vector<float> ParseStringToFloats(const std::string& input) {
	std::vector<float> numbers;
	std::stringstream ss(input);
	std::string token;

	while (std::getline(ss, token, ',')) {
		std::stringstream tokenStream(token);
		float value;
		while (tokenStream >> value) {
			numbers.push_back(value);
			if (tokenStream.peek() == ' ') {
				tokenStream.ignore();
			}
		}
	}

	return numbers;
}

int MujocoXmlParser::ProcessBodyElement(
	BodyParam* _parentBody,
	std::vector<float> _parentPosVec,
	std::vector<float> _parentLocalPosVec,
	const XMLElement* _bodyElement, int depth) 
{
	if (0 == _bodyElement) {
		return -1;
	}

	BodyParam* _bodyParam = new BodyParam();
	if (_parentBody->pChild)
	{
		BodyParam* _whereToPut = _parentBody->pChild;

		while (0 != _whereToPut->pSibling)
		{
			_whereToPut = _whereToPut->pSibling;
		}
		_whereToPut->pSibling = _bodyParam;
	}
	else
	{
		_parentBody->pChild = _bodyParam;
	}
	
	_bodyParam->pParent = _parentBody;

	const char* _bodyPosStr = _bodyElement->Attribute("pos");
	std::vector<float> _bodyLocalPosVec = ParseStringToFloats(_bodyPosStr);
	std::vector<float> _bodyPosVec(_bodyLocalPosVec);

	_bodyParam->posLocal[0] = _bodyLocalPosVec[0] * this->fScale;
	_bodyParam->posLocal[1] = _bodyLocalPosVec[1] * this->fScale;
	_bodyParam->posLocal[2] = _bodyLocalPosVec[2] * this->fScale;

	_bodyPosVec[0] += _parentPosVec[0];
	_bodyPosVec[1] += _parentPosVec[1];
	_bodyPosVec[2] += _parentPosVec[2];

	_bodyParam->posGlobal[0] = _bodyPosVec[0] * this->fScale;
	_bodyParam->posGlobal[1] = _bodyPosVec[1] * this->fScale;
	_bodyParam->posGlobal[2] = _bodyPosVec[2] * this->fScale;

	GeomParam* _pPrevGeomParam = 0;

	for (const XMLElement* childBodyElement = _bodyElement->FirstChildElement("geom");
		childBodyElement != nullptr;
		childBodyElement = childBodyElement->NextSiblingElement("geom")) 
	{
		if (nullptr == childBodyElement) continue;

		GeomParam* _pGeomParam = new GeomParam();

		if (0 != _pPrevGeomParam)
		{
			_pPrevGeomParam->pNext = _pGeomParam;
		}
		else
		{
			_bodyParam->pGeoms = _pGeomParam;	
		}

		_pGeomParam->pPrev = _pPrevGeomParam;
		_pPrevGeomParam = _pGeomParam;


		char* geomType = "";
		const char* _geomPosStr = childBodyElement->Attribute("pos");
		geomType = (char*)childBodyElement->Attribute("type");

		if (NULL == geomType)
		{
			// Todo: Shall use default geom type, hard coded as capsule temporarily
			geomType = "capsule";
		}

		strcpy(_pGeomParam->strType, geomType);

		if (0 == strcmp(geomType, "capsule"))
		{
			// Default shape
			const char* _sizeStr = childBodyElement->Attribute("size");
			float _radius = atof(_sizeStr);

			_pGeomParam->fRadius = _radius * this->fScale;

			const char* _shapeStr = childBodyElement->Attribute("fromto");
			std::vector<float> _shapeVec = ParseStringToFloats(_shapeStr);

			float halfHeight = pow((
				pow((_shapeVec[3] - _shapeVec[0]), 2.0f)
				+ pow((_shapeVec[4] - _shapeVec[1]), 2.0f)
				+ pow((_shapeVec[5] - _shapeVec[2]), 2.0f)), 0.5f) / 2.0f;

			_pGeomParam->fHalfHeight = halfHeight * this->fScale;

			_pGeomParam->fPos[0] = (_shapeVec[3] + _shapeVec[0]) / 2.0f * this->fScale;
			_pGeomParam->fPos[1] = (_shapeVec[4] + _shapeVec[1]) / 2.0f * this->fScale;
			_pGeomParam->fPos[2] = (_shapeVec[5] + _shapeVec[2]) / 2.0f * this->fScale;

			float _dir_mod_x = fabs(_shapeVec[3] - _shapeVec[0]);
			float _dir_mod_y = fabs(_shapeVec[4] - _shapeVec[1]);
			float _dir_mod_z = fabs(_shapeVec[5] - _shapeVec[2]);

			if (_dir_mod_x > _dir_mod_y && _dir_mod_x > _dir_mod_z) {
				_pGeomParam->qOrient[0] = 0;
			}
			else if (_dir_mod_y > _dir_mod_x && _dir_mod_y > _dir_mod_z) {
				_pGeomParam->qOrient[0] = 90.0f;
				_pGeomParam->qOrient[1] = 0.0f;
				_pGeomParam->qOrient[2] = 0.0f;
				_pGeomParam->qOrient[3] = 1.0f;
			}
			else {
				_pGeomParam->qOrient[0] = 90.0f;
				_pGeomParam->qOrient[1] = 0.0f;
				_pGeomParam->qOrient[2] = 1.0f;
				_pGeomParam->qOrient[3] = 0.0f;
			}
		}
		else
		{
			if (0 == strcmp(geomType, "sphere"))
			{
				const char* _sizeStr = childBodyElement->Attribute("size");
				float _radius = atof(_sizeStr) * this->fScale;

				_pGeomParam->fRadius = _radius;
			}
			else if (0 == strcmp(geomType, "box"))
			{
				const char* _sizeStr = childBodyElement->Attribute("size");
				std::vector<float> _sizeVec = ParseStringToFloats(_sizeStr);

				_pGeomParam->fSize[0] = _sizeVec[0] * this->fScale;
				_pGeomParam->fSize[1] = _sizeVec[1] * this->fScale;
				_pGeomParam->fSize[2] = _sizeVec[2] * this->fScale;
			}
			else
			{
				//Todo: not supported yet.
			}

			if (0 == _geomPosStr)
			{
				_pGeomParam->fPos[0] = 0.0f;
				_pGeomParam->fPos[1] = 0.0f;
				_pGeomParam->fPos[2] = 0.0f;
			}
			else
			{
				std::vector<float> _shapeVec = ParseStringToFloats(_geomPosStr);
				_pGeomParam->fPos[0] = _shapeVec[0] * this->fScale;
				_pGeomParam->fPos[1] = _shapeVec[1] * this->fScale;
				_pGeomParam->fPos[2] = _shapeVec[2] * this->fScale;
			}
		}
	}

	bool _containJoint = false;

	JointParam* _pPrevJointParam = 0;

	for (const XMLElement* childBodyElement = _bodyElement->FirstChildElement("joint");
		childBodyElement != nullptr;
		childBodyElement = childBodyElement->NextSiblingElement("joint"))
	{
		_containJoint = true;

		JointParam* _pJointParam = new JointParam();
		_pJointParam->pPrev = _pPrevJointParam;

		const char* _str = childBodyElement->Attribute("axis");
		std::vector<float> _axis = ParseStringToFloats(_str);
		_pJointParam->axis[0] = _axis[0];
		_pJointParam->axis[1] = _axis[1];
		_pJointParam->axis[2] = _axis[2];

		_str = childBodyElement->Attribute("range");
		std::vector<float> _range = ParseStringToFloats(_str);
		_pJointParam->range[0] = _range[0];
		_pJointParam->range[1] = _range[1];

		_str = childBodyElement->Attribute("stiffness");
		_pJointParam->stiffness = atof(_str);

		_str = childBodyElement->Attribute("damping");
		_pJointParam->damping = atof(_str);

		_str = childBodyElement->Attribute("armature");
		_pJointParam->armature = atof(_str);

		if (0 != _pPrevJointParam)
		{
			_pPrevJointParam->pNext = _pJointParam;
		}
		else
		{
			_bodyParam->pJoints = _pJointParam;
		}

		_pPrevJointParam = _pJointParam;
	}


	string indent(depth * 2, ' ');  // Create an indent string for better readability
	const char* bodyName = _bodyElement->Attribute("name");
	std::cout << indent << "Body Name: " << (bodyName != nullptr ? bodyName : "unnamed")
		<< ", Position: " << (_bodyPosStr != nullptr ? _bodyPosStr : "undefined") << endl;

	// Recursively process child bodies
	for (const XMLElement* childBodyElement = _bodyElement->FirstChildElement("body");
		childBodyElement != nullptr;
		childBodyElement = childBodyElement->NextSiblingElement("body")) {

		ProcessBodyElement(_bodyParam, _bodyPosVec, _bodyLocalPosVec, childBodyElement, depth + 1);
	}

	return 0;
}

BodyParam* MujocoXmlParser::Load(const char* pXmlFileName, float* _posRoot, float _scale)
{
	this->fScale = _scale;

	XMLDocument doc;
	XMLError eResult = doc.LoadFile(pXmlFileName);
	if (eResult != XML_SUCCESS) {
		cout << "Error loading file!" << endl;
		return 0;
	}

	XMLNode* pRoot = doc.FirstChild();
	if (pRoot == nullptr) {
		cout << "No root found!" << endl;
		return 0;
	}

	XMLElement* pElement = pRoot->FirstChildElement("worldbody");
	if (pElement == nullptr) {
		cout << "No worldbody found!" << endl;
		return 0;
	}

	std::vector<float> _bodyPosVec;
	_bodyPosVec.push_back(0.0f);
	_bodyPosVec.push_back(0.0f);
	_bodyPosVec.push_back(0.0f);

	std::vector<float> _bodyLocalPosVec;
	if (0 == _posRoot)
	{
		_bodyLocalPosVec.push_back(0.0f);
		_bodyLocalPosVec.push_back(0.0f);
		_bodyLocalPosVec.push_back(0.0f);
	}
	else
	{
		_bodyLocalPosVec.push_back(_posRoot[0]);
		_bodyLocalPosVec.push_back(_posRoot[1]);
		_bodyLocalPosVec.push_back(_posRoot[2]);
	}

	BodyParam* pRootBody = new BodyParam();

	for (XMLElement* bodyElement = pElement->FirstChildElement("body");
		bodyElement != nullptr; bodyElement = bodyElement->NextSiblingElement("body")) {
		const char* bodyName = bodyElement->Attribute("name");
		const char* bodyPos = bodyElement->Attribute("pos");
		cout << "Body Name: " << (bodyName != nullptr ? bodyName : "unnamed") << ", Position: " << (bodyPos != nullptr ? bodyPos : "undefined") << endl;
		if (bodyElement)
		{
			ProcessBodyElement(pRootBody, _bodyPosVec, _bodyLocalPosVec, bodyElement, 0);
			break;
		}
	}

	return pRootBody;

}
