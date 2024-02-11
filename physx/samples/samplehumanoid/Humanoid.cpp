//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2021 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#include "MujocoXmlParser.h"


#include "PxPhysicsAPI.h"
#include "extensions/PxExtensionsAPI.h"
#include "SampleHumanoid.h"
#include "Humanoid.h"
#include "RendererColor.h"
#include "RenderPhysX3Debug.h"

#include "PxTkStream.h"

#include "PhysXSample.h"
#include "PxTkFile.h"
using namespace PxToolkit;
// if enabled: runs the crab AI in sync, not as a parallel task to physx.
#define DEBUG_RENDERING 0
/*
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
*/



// table with default times in seconds how the crab AI will try to stay in a state
static const PxReal gDefaultStateTime[HumanoidState::eNUM_STATES] = {5.0f, 10.0f, 10.0f, 10.0f, 10.0f, 6.0f};

Humanoid::Humanoid(SampleHumanoid& sample, const PxVec3& crabPos, RenderMaterial* material)
: ClassType(ClassType::eHUMANOID)
, mSampleHumanoid(&sample)
, mMaterial(material)
{
	initMembers();
	create(crabPos);
}

Humanoid::Humanoid(SampleHumanoid& sample, const char* filename, RenderMaterial* material)
: ClassType(ClassType::eHUMANOID)
, mSampleHumanoid(&sample)
, mMaterial(material)
{
	initMembers();
	load(filename);
}

void Humanoid::initMembers()
{
	mMemory = NULL;
	mHumanoidBody = NULL;

	mLegHeight = 0;
	mRespawnMe = false;
	mHumanoidState = HumanoidState::eWAITING;
	mStateTime = gDefaultStateTime[HumanoidState::eWAITING];
	mAccumTime = 0;
	mElapsedTime = 0;
	mRunning = 0;

	mAcceleration[0] = 0;
	mAcceleration[1] = 0;

	mAccelerationBuffer[0] = 0;
	mAccelerationBuffer[1] = 0;

	mHumanoidPos = PxVec3(0);

}
Humanoid::~Humanoid()
{
	// wait until background task is finished
	while(mRunning)
		;

	for(PxU32 i = 0; i < mJoints.size(); i++)
		mJoints[i]->release();
	mJoints.clear();

	for(PxU32 i = 0; i < mActors.size(); i++)
		mSampleHumanoid->removeActor(mActors[i]);
	mActors.clear();
	
	if(mMemory)
	{
		SAMPLE_FREE(mMemory);
	}

}

static void setShapeFlag(PxRigidActor* actor, PxShapeFlag::Enum flag, bool flagValue)
{
	const PxU32 numShapes = actor->getNbShapes();
	PxShape** shapes = (PxShape**)SAMPLE_ALLOC(sizeof(PxShape*)*numShapes);
	actor->getShapes(shapes, numShapes);
	for(PxU32 i = 0; i < numShapes; i++)
	{
		PxShape* shape = shapes[i];
		shape->setFlag(flag, flagValue);
	}
	SAMPLE_FREE(shapes);
}

PxVec3 Humanoid::getPlaceOnFloor(PxVec3 start)
{
	PxRaycastBuffer rayHit;
	mSampleHumanoid->getActiveScene().raycast(start, PxVec3(0,-1,0), 1000.0f, rayHit);

	return rayHit.block.position + PxVec3(0,mLegHeight,0);
}

static const PxSerialObjectId mMaterial_id		= (PxSerialObjectId)0x01;
static const PxSerialObjectId mHumanoidBody_id		= (PxSerialObjectId)0x02;
static const PxSerialObjectId mMotorJoint0_id	= (PxSerialObjectId)0x03;
static const PxSerialObjectId mMotorJoint1_id	= (PxSerialObjectId)0x04; 

void Humanoid::save(const char* filename)
{
	PxPhysics& physics = mSampleHumanoid->getPhysics();
	PxCollection* thePxCollection = PxCreateCollection();
	PxSerializationRegistry* sr = PxSerialization::createSerializationRegistry(physics);	
	for(PxU32 i = 0; i < mActors.size(); ++i)
	{		
		thePxCollection->add(*mActors[i]);
	}
	
	for(PxU32 i = 0; i < mJoints.size(); ++i)
	{
		thePxCollection->add(*mJoints[i]);
	}
	
	thePxCollection->addId(*mHumanoidBody,	mHumanoidBody_id);
	thePxCollection->addId(*mMotorJoint[0],  mMotorJoint0_id);
	thePxCollection->addId(*mMotorJoint[1],  mMotorJoint1_id);
	
	PxCollection* theExtRef = PxCreateCollection();
	theExtRef->add(*mSampleHumanoid->mMaterial, mMaterial_id);
	
	PxSerialization::complete(*thePxCollection, *sr, theExtRef);

	PxDefaultFileOutputStream s(filename);
	PxSerialization::serializeCollectionToBinary(s, *thePxCollection, *sr, theExtRef);
	
	theExtRef->release();
	thePxCollection->release();
	sr->release();
}

static PxU32 GetFileSize(const char* name)
{
	if(!name)	return 0;

#ifndef SEEK_END
#define SEEK_END 2
#endif

	SampleFramework::File* fp;
	if (PxToolkit::fopen_s(&fp, name, "rb"))
		return 0;
	fseek(fp, 0, SEEK_END);
	PxU32 eof_ftell = (PxU32)ftell(fp);
	fclose(fp);
	return eof_ftell;
}

void Humanoid::load(const char* filename)
{
	
	PxPhysics& thePhysics = mSampleHumanoid->getPhysics();

	SampleFramework::File* fp = NULL;
	if (!PxToolkit::fopen_s(&fp, filename, "rb"))
	{
		PxU32 theFileSize = GetFileSize(filename);
		
		if(!mMemory)
			mMemory = SAMPLE_ALLOC(theFileSize + PX_SERIAL_FILE_ALIGN);

		void* theMemory16 = (void*)((size_t(mMemory) + PX_SERIAL_FILE_ALIGN)&~(PX_SERIAL_FILE_ALIGN-1));
		const size_t theNumRead = fread(theMemory16, 1, theFileSize, fp);
		PX_ASSERT(PxU32(theNumRead) == theFileSize);
		PX_UNUSED(theNumRead);
		fclose(fp);

		PxCollection* theExtRef = PxCreateCollection();
		theExtRef->add(*mSampleHumanoid->mMaterial, mMaterial_id);
		PxSerializationRegistry* sr = PxSerialization::createSerializationRegistry(thePhysics);
		PxCollection* thePxCollection = PxSerialization::createCollectionFromBinary(theMemory16, *sr, theExtRef);
		PX_ASSERT(thePxCollection);

		mSampleHumanoid->getActiveScene().addCollection(*thePxCollection);
			
		mMotorJoint[0] = reinterpret_cast<PxRevoluteJoint*>( thePxCollection->find(mMotorJoint0_id));
		mMotorJoint[1] = reinterpret_cast<PxRevoluteJoint*>( thePxCollection->find(mMotorJoint1_id));
		mHumanoidBody = reinterpret_cast<PxRigidDynamic*>( thePxCollection->find(mHumanoidBody_id));
		PX_ASSERT(mMotorJoint[0] && mMotorJoint[1] && mHumanoidBody );

		PxU32 nbObjs = thePxCollection->getNbObjects();
		PX_ASSERT(nbObjs != 0);
		for(PxU32 i = 0; i < nbObjs; ++i)
		{
			PxBase* object = &thePxCollection->getObject(i);
			if(object)
			{
				const PxType serialType = object->getConcreteType();
				if(serialType == PxConcreteType::eRIGID_DYNAMIC)
				{
					PxRigidDynamic* actor = reinterpret_cast<PxRigidDynamic*>(object);

					mSampleHumanoid->createRenderObjectsFromActor(actor , mMaterial ); 
					mSampleHumanoid->addPhysicsActors( actor );
					mActors.push_back( actor );
				}
				else if(serialType == PxConcreteType::eCONSTRAINT)
				{
					PxU32 typeID = 0;
					PxConstraint* constraint = reinterpret_cast<PxConstraint*>(object);
					PxJoint* joint = reinterpret_cast<PxJoint*>(constraint->getExternalReference(typeID));
					mJoints.push_back( joint );
				}
				else if(serialType == PxConcreteType::eSHAPE)
				{
					//giving up application shape ownership early
					PxShape* shape = reinterpret_cast<PxShape*>(object);
					shape->release();
				}
			}
		}
		
		theExtRef->release();
		thePxCollection->release();
		sr->release();
	}

	if( !mHumanoidBody ) mSampleHumanoid->fatalError( "createBox failed!" );

}

#define CREATE_MUJOCO
void Humanoid::create(const PxVec3& _crabPos)
{
	static const PxReal baseSize = 1.0f;
	static const PxVec3 crabBodyDim = PxVec3(baseSize, baseSize, baseSize);

#ifdef CREATE_MUJOCO
	loadMujoco();

#else

	/*
	PxShape* shape;
	mHumanoidBody->getShapes(&shape, 1);
	shape->setLocalPose(PxTransform(PxQuat(PxHalfPi * 0.0f, PxVec3(0, 0, 1))));
	PxRigidBodyExt::setMassAndUpdateInertia(*mHumanoidBody, legMass * 10.0f);
	PxTransform cmPose = mHumanoidBody->getCMassLocalPose();
//	cmPose.p.y -= 0.8f;
	mHumanoidBody->setCMassLocalPose(cmPose);
	mHumanoidBody->setAngularDamping(1.0f);
	mHumanoidBody->userData = this;
	*/

	
	/*
	
	*/
	mLegHeight = baseSize;
	PxVec3 crabPos = getPlaceOnFloor(_crabPos);
	mHumanoidBody = mSampleHumanoid->createBox(crabPos + PxVec3(0, 0, 0), crabBodyDim, NULL, mMaterial, 1000.0f, true)->is<PxRigidDynamic>();

	if (!mHumanoidBody) {
		mSampleHumanoid->fatalError("createBox failed!");
	}

	setShapeFlag(mHumanoidBody, PxShapeFlag::eSIMULATION_SHAPE, false);
	mActors.push_back(mHumanoidBody);
	
	// legs
	PxVec3 bodyToLegPos0 = PxVec3(0, 0, 0);
	PxVec3 motorToLegPos0 = PxVec3(0, 0, 0);
	PxReal angle0 = 0;


	createTorso(mHumanoidBody, bodyToLegPos0, NULL, motorToLegPos0);
#endif	
}


void Humanoid::createLeg(PxRigidDynamic* mainBody, PxVec3 localPos, PxReal mass, PxReal scale, PxRigidDynamic* motor, PxVec3 motorAttachmentPos)
{
	float params_a = 0.5f;
	float params_b = 0.6f;
	float params_c = 0.5f;
	float params_d = 0.5f;
	float params_e = 1.5f;
	float params_m = 0.3f;
	float params_n = 0.1f;

	PxVec3 crabLegPos = mainBody->getGlobalPose().p + localPos;

	// params for Theo Jansen's machine
	// check edge ascii art in Crab.h
	static const PxReal stickExt = 0.125f * 0.5f * scale;
	const PxReal a = params_a * scale;
	const PxReal b = params_b * scale;
	const PxReal c = params_c * scale;
	const PxReal d = params_d * scale;
	const PxReal e = params_e * scale;
	const PxReal m = params_m * scale;
	const PxReal n = params_n * scale;

	const PxReal density = 1.0f;

	std::vector<PxTransform> poses;
	std::vector<const PxGeometry*> geometries;

	PxBoxGeometry boxGeomA = PxBoxGeometry(a, stickExt, stickExt);
	PxBoxGeometry boxGeomB = PxBoxGeometry(stickExt, b, stickExt);
	PxBoxGeometry boxGeomC = PxBoxGeometry(stickExt, c, stickExt);

	PxCapsuleGeometry capsGeomD = PxCapsuleGeometry(stickExt * 2.0f, d);

	for (PxU32 leg = 0; leg < 2; leg++)
	{
		bool left = (leg == 0);
#define MIRROR(X) left ? -1.0f*(X) : (X)
		PxVec3 startPos = crabLegPos + PxVec3(MIRROR(e), 0, 0);

		// create upper triangle from boxes
		PxRigidDynamic* upperTriangle = NULL;
		{
			PxTransform poseA = PxTransform(PxVec3(MIRROR(a), 0, 0));
			PxTransform poseB = PxTransform(PxVec3(MIRROR(0), b, 0));
			poses.clear(); geometries.clear();
			poses.push_back(poseA); poses.push_back(poseB);
			geometries.push_back(&boxGeomA); geometries.push_back(&boxGeomB);
			upperTriangle = mSampleHumanoid->createCompound(startPos, poses, geometries, NULL, mMaterial, density)->is<PxRigidDynamic>();
			if (!upperTriangle) mSampleHumanoid->fatalError("createCompound failed!");
			mActors.push_back(upperTriangle);
		}

		// create lower triangle from boxes
		PxRigidDynamic* lowerTriangle = NULL;
		{
			PxTransform poseA = PxTransform(PxVec3(MIRROR(a), 0, 0));
			//PxTransform poseD = PxTransform(PxVec3(MIRROR(0), -d, 0));
			PxTransform poseD = PxTransform(PxVec3(MIRROR(0), -d, 0), PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
			poses.clear(); geometries.clear();
			poses.push_back(poseA); poses.push_back(poseD);
			//geometries.push_back(&boxGeomA); geometries.push_back(&boxGeomD);
			geometries.push_back(&boxGeomA); geometries.push_back(&capsGeomD);
			lowerTriangle = mSampleHumanoid->createCompound(startPos + PxVec3(0, -2.0f * c, 0), poses, geometries, NULL, mMaterial, density)->is<PxRigidDynamic>();
			if (!lowerTriangle) mSampleHumanoid->fatalError("createCompound failed!");
			mActors.push_back(lowerTriangle);
		}

		// create vertical boxes to connect the triangles
		PxRigidDynamic* verticalBox0 = mSampleHumanoid->createBox(startPos + PxVec3(0, -c, 0), boxGeomC.halfExtents, NULL, mMaterial, density)->is<PxRigidDynamic>();
		if (!verticalBox0) mSampleHumanoid->fatalError("createBox failed!");
		PxRigidDynamic* verticalBox1 = mSampleHumanoid->createBox(startPos + PxVec3(MIRROR(2.0f * a), -c, 0), boxGeomC.halfExtents, NULL, mMaterial, density)->is<PxRigidDynamic>();
		if (!verticalBox1) mSampleHumanoid->fatalError("createBox failed!");
		mActors.push_back(verticalBox0);
		mActors.push_back(verticalBox1);

		// disable gravity
		upperTriangle->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
		lowerTriangle->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
		verticalBox0->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
		verticalBox1->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);

		// set mass
		PxRigidBodyExt::setMassAndUpdateInertia(*upperTriangle, mass);
		PxRigidBodyExt::setMassAndUpdateInertia(*lowerTriangle, mass);
		PxRigidBodyExt::setMassAndUpdateInertia(*verticalBox0, mass);
		PxRigidBodyExt::setMassAndUpdateInertia(*verticalBox1, mass);

		// turn off collision upper triangle and vertical boxes
		setShapeFlag(upperTriangle, PxShapeFlag::eSIMULATION_SHAPE, false);
		setShapeFlag(verticalBox0, PxShapeFlag::eSIMULATION_SHAPE, false);
		setShapeFlag(verticalBox1, PxShapeFlag::eSIMULATION_SHAPE, false);

		// revolute joint in lower corner of upper triangle
		PxRevoluteJoint* joint;
		joint = PxRevoluteJointCreate(mSampleHumanoid->getPhysics(),
			mainBody, PxTransform(PxVec3(MIRROR(e), 0, 0) + localPos, PxQuat(-PxHalfPi, PxVec3(0, 1, 0))),
			upperTriangle, PxTransform(PxVec3(0, 0, 0), PxQuat(-PxHalfPi, PxVec3(0, 1, 0))));
		if (!joint) mSampleHumanoid->fatalError("PxRevoluteJointCreate failed!");
		mJoints.push_back(joint);

		// 4 revolute joints to connect triangles
		joint = PxRevoluteJointCreate(mSampleHumanoid->getPhysics(),
			upperTriangle, PxTransform(PxVec3(0, 0, 0), PxQuat(-PxHalfPi, PxVec3(0, 1, 0))),
			verticalBox0, PxTransform(PxVec3(0, c, 0), PxQuat(-PxHalfPi, PxVec3(0, 1, 0))));
		if (!joint) mSampleHumanoid->fatalError("PxRevoluteJointCreate failed!");
		mJoints.push_back(joint);

		joint = PxRevoluteJointCreate(mSampleHumanoid->getPhysics(),
			upperTriangle, PxTransform(PxVec3(MIRROR(2.0f * a), 0, 0), PxQuat(-PxHalfPi, PxVec3(0, 1, 0))),
			verticalBox1, PxTransform(PxVec3(0, c, 0), PxQuat(-PxHalfPi, PxVec3(0, 1, 0))));
		if (!joint) mSampleHumanoid->fatalError("PxRevoluteJointCreate failed!");
		mJoints.push_back(joint);

		joint = PxRevoluteJointCreate(mSampleHumanoid->getPhysics(),
			lowerTriangle, PxTransform(PxVec3(0, 0, 0), PxQuat(-PxHalfPi, PxVec3(0, 1, 0))),
			verticalBox0, PxTransform(PxVec3(0, -c, 0), PxQuat(-PxHalfPi, PxVec3(0, 1, 0))));
		if (!joint) mSampleHumanoid->fatalError("PxRevoluteJointCreate failed!");
		mJoints.push_back(joint);

		joint = PxRevoluteJointCreate(mSampleHumanoid->getPhysics(),
			lowerTriangle, PxTransform(PxVec3(MIRROR(2.0f * a), 0, 0), PxQuat(-PxHalfPi, PxVec3(0, 1, 0))),
			verticalBox1, PxTransform(PxVec3(0, -c, 0), PxQuat(-PxHalfPi, PxVec3(0, 1, 0))));
		if (!joint) mSampleHumanoid->fatalError("PxRevoluteJointCreate failed!");
		mJoints.push_back(joint);

		// 2 distance constraints to connect motor with the triangles
		PxTransform motorTransform = PxTransform(motorAttachmentPos);
		PxReal dist0 = PxSqrt((2.0f * b - n) * (2.0f * b - n) + (e - m) * (e - m));
		PxReal dist1 = PxSqrt((2.0f * c + n) * (2.0f * c + n) + (e - m) * (e - m));

		PxDistanceJoint* distJoint0 = PxDistanceJointCreate(mSampleHumanoid->getPhysics(), upperTriangle, PxTransform(PxVec3(0, 2.0f * b, 0)), motor, motorTransform);
		if (!distJoint0) mSampleHumanoid->fatalError("PxDistanceJointCreate failed!");
		// set min & max distance to dist0
		distJoint0->setMaxDistance(dist0);
		distJoint0->setMinDistance(dist0);
		// setup damping & spring
		distJoint0->setDamping(0.1f);
		distJoint0->setStiffness(100.0f);
		distJoint0->setDistanceJointFlags(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED | PxDistanceJointFlag::eMIN_DISTANCE_ENABLED | PxDistanceJointFlag::eSPRING_ENABLED);

		PxDistanceJoint* distJoint1 = PxDistanceJointCreate(mSampleHumanoid->getPhysics(), lowerTriangle, PxTransform(PxVec3(0, 0, 0)), motor, motorTransform);
		if (!distJoint1) mSampleHumanoid->fatalError("PxDistanceJointCreate failed!");
		// set min & max distance to dist0
		distJoint1->setMaxDistance(dist1);
		distJoint1->setMinDistance(dist1);
		// setup damping & spring
		distJoint1->setDamping(0.1f);
		distJoint1->setStiffness(100.0f);
		distJoint1->setDistanceJointFlags(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED | PxDistanceJointFlag::eMIN_DISTANCE_ENABLED | PxDistanceJointFlag::eSPRING_ENABLED);

		// one distance joint to ensure that the vertical boxes do not get stuck if they cross the diagonal.
		PxReal halfDiagDist = PxSqrt(a * a + c * c);
		PxDistanceJoint* noFlip = PxDistanceJointCreate(mSampleHumanoid->getPhysics(), lowerTriangle, PxTransform(PxVec3(MIRROR(2.0f * a), 0, 0)), upperTriangle, PxTransform(PxVec3(0)));
		if (!noFlip) mSampleHumanoid->fatalError("PxDistanceJointCreate failed!");
		// set min & max distance to dist0
		noFlip->setMaxDistance(2.0f * (a + c));
		noFlip->setMinDistance(halfDiagDist);
		// setup damping & spring
		noFlip->setDamping(1.0f);
		noFlip->setStiffness(100.0f);
		noFlip->setDistanceJointFlags(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED | PxDistanceJointFlag::eMIN_DISTANCE_ENABLED | PxDistanceJointFlag::eSPRING_ENABLED);

		mJoints.push_back(distJoint0);
		mJoints.push_back(distJoint1);
		mJoints.push_back(noFlip);
	}
}

std::vector<float> parseStringToFloats(const std::string& input) {
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

void SetFilterData(PxRigidDynamic* _dynamic, int _filter_data)
{
	PxFilterData filterData1;

	// Assuming CATEGORY_1 is a predefined category for these types of objects
	filterData1.word2 = _filter_data; // Category bits

	// Apply filter data to the shapes of each body
	PxShape* shape; 
	_dynamic->getShapes(&shape, 1);
	shape->setSimulationFilterData(filterData1);
}

int GetFilterData(PxRigidDynamic* _dynamic)
{
	PxFilterData filterData1;

	// Apply filter data to the shapes of each body
	PxShape* shape;
	_dynamic->getShapes(&shape, 1);
	filterData1 = shape->getSimulationFilterData();
	int _ret = filterData1.word2;
	return _ret;
}

void Humanoid::ProcessMujoco(BodyParam* _bodyParam, PxRigidDynamic* _bodyParent)
{
	static float _minival = 1e-3;
	if (0 == _bodyParam) return;

	BodyParam* pNow = _bodyParam;
	while (0 != pNow)
	{
		std::vector<PxTransform> _geomPoses;
		std::vector<const PxGeometry*> _geometries;
		GeomParam* _pGeom = pNow->pGeoms;
		if (0 == _pGeom)
		{
			// Short return;
			pNow = pNow->pChild;
			continue;
		}

		while (0 != _pGeom)
		{
			if (0 == strcmp(_pGeom->strType, "capsule"))
			{
				float _radius = _pGeom->fRadius;
				float halfHeight = _pGeom->fHalfHeight;

				PxCapsuleGeometry* _geom = new PxCapsuleGeometry(_radius, halfHeight);
				_geometries.push_back(_geom);
			}
			else if (0 == strcmp(_pGeom->strType, "sphere"))
			{
				float _radius = _pGeom->fRadius;

				PxSphereGeometry* _geom = new PxSphereGeometry(_radius);
				_geometries.push_back(_geom);
			}
			else if (0 == strcmp(_pGeom->strType, "box"))
			{
				PxBoxGeometry* _geom = new PxBoxGeometry(_pGeom->fSize[0], _pGeom->fSize[1], _pGeom->fSize[2]);
				_geometries.push_back(_geom);
			}

			PxTransform _geomPos;
			PxQuat _qRotate;
			if (fabs(_pGeom->qOrient[0]) > _minival)
			{
				_qRotate = PxQuat(PxPi * _pGeom->qOrient[0] / 180.0f, PxVec3(_pGeom->qOrient[1], _pGeom->qOrient[2], _pGeom->qOrient[3]));
				_geomPos = PxTransform(PxVec3(_pGeom->fPos[0], _pGeom->fPos[1], _pGeom->fPos[2]),
					_qRotate);
			}
			else
			{
				_geomPos = PxTransform(PxVec3(_pGeom->fPos[0], _pGeom->fPos[1], _pGeom->fPos[2]));
			}

			_geomPoses.push_back(_geomPos);

			GeomParam* pNexGeom = _pGeom->pNext;
			// Delete the geom
			_pGeom = pNexGeom;
		}

		physx::PxVec3 _bodyGlobalPos = physx::PxVec3(pNow->posGlobal[0], pNow->posGlobal[1], pNow->posGlobal[2]);
		physx::PxRigidDynamic* _bodyPart = mSampleHumanoid->createCompound(_bodyGlobalPos, _geomPoses, _geometries, NULL, mMaterial, 1.0f)->is<PxRigidDynamic>();
		if (!_bodyPart) mSampleHumanoid->fatalError("createCompound failed!");
		mActors.push_back(_bodyPart);
		for (const PxGeometry* element : _geometries) {
			delete element;
		}
		_geometries.clear();
		_geomPoses.clear();

		// Assuming you have a pointer to a PxRigidActor or PxRigidDynamic named 'actor'
		physx::PxQuat rotationQuat = physx::PxQuat(physx::PxPi * -0.5f, physx::PxVec3(1, 0, 0)); // 90 degrees around Y axis

	//	_bodyPart->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
	//	_bodyPart->setGlobalPose(physx::PxTransform(_bodyPart->getGlobalPose().p, rotationQuat), true);
	//	_bodyPart->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

		_bodyPart->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		// Now set the kinematic target
		PxVec3 rotatedRelativePos = rotationQuat.rotate(_bodyPart->getGlobalPose().p);
		_bodyPart->setKinematicTarget(physx::PxTransform(rotatedRelativePos, rotationQuat));


		if (NULL == mHumanoidBody) {
			mHumanoidBody = _bodyPart;
			SetFilterData(_bodyPart, 32);

			physx::PxVec3 initialLinearVelocity(0.0f, 0.0f, 0.0f);
			mHumanoidBody->setLinearVelocity(initialLinearVelocity);

			physx::PxVec3 initialAngularVelocity(0.0f, 0.0f, 0.0f);
			mHumanoidBody->setAngularVelocity(initialAngularVelocity);
		}
		else
		{
			if (_bodyParent)
			{
				int _parent_body_filter_data = GetFilterData(_bodyParent);
				int _filter_data = _parent_body_filter_data << 1;
				SetFilterData(_bodyPart, _filter_data);
			}
			else
			{
				assert(false);
			}
		}

		JointParam _swing1_param;
		JointParam _swing2_param;
		JointParam _twist_param;
		bool _has_swing1 = false;
		bool _has_swing2 = false;
		bool _has_twist = false;

		JointParam* _pJoint = pNow->pJoints;
		while (0 != _pJoint)
		{
			if (_pJoint->axis[1] > 0.9)
			{
				_has_swing1 = true;
				_swing1_param = *_pJoint;
			}
			else if (_pJoint->axis[0] > 0.9)
			{
				_has_swing2 = true;
				_swing2_param = *_pJoint;
			}
			else if (_pJoint->axis[2] > 0.9)
			{
				_has_twist = true;
				_twist_param = *_pJoint;
			}

			JointParam* pNexJoint = _pJoint->pNext;
			// Delete the geom
			_pJoint = pNexJoint;
		}

		// Create Joints
		PxJointLimitPyramid* p_swingLimit = 0;
		PxJointLimitCone* p_swing1Limit = 0;
		PxJointLimitCone* p_swing2Limit = 0;
		PxJointAngularLimitPair* p_twistLimit = 0;

		if (_has_swing1 && _has_swing2)
		{
			float _y_min = std::max<float>((_swing1_param.range[0] / 180.0f) * PxPi - _minival, -PxPi + _minival);
			float _y_max = std::min((_swing1_param.range[1] / 180.0f) * PxPi + _minival, PxPi - _minival);
			float _z_min = std::max((_swing2_param.range[0] / 180.0f) * PxPi - _minival, -PxPi + _minival);
			float _z_max = std::min((_swing2_param.range[1] / 180.0f) * PxPi + _minival, PxPi - _minival);

			p_swingLimit = new PxJointLimitPyramid(
				_y_min,
				_y_max,
				_z_min,
				_z_max);
		}
		else
		{
			if (_has_swing1)
			{
				float _swing1_limit = std::min((_swing1_param.range[1] / 180.0f) * PxPi + _minival, PxPi - _minival);
				p_swing1Limit = new PxJointLimitCone(_swing1_limit, _minival, 0);
			}
			else if (_has_swing2)
			{
				float _swing2_limit = std::min((_swing2_param.range[1] / 180.0f) * PxPi + _minival, PxPi - _minival);
				p_swing2Limit = new PxJointLimitCone(_minival, _swing2_limit, 0);
			}
		}

		// Set twist limits
		if (_has_twist)
		{
			p_twistLimit = new PxJointAngularLimitPair(
				max((_twist_param.range[0] / 180.0f) * PxPi - _minival, -PxPi + _minival),
				min((_twist_param.range[1] / 180.0f) * PxPi + _minival, PxPi - _minival));
		}

		physx::PxVec3 _bodyLocalPos = physx::PxVec3(pNow->posLocal[0], pNow->posLocal[1], pNow->posLocal[2]);
		PxD6Joint* _joint = NULL;
		// PxQuat(PxHalfPi, PxVec3(0, 0, 1))
		_joint = PxD6JointCreate(mSampleHumanoid->getPhysics(),
			_bodyParent,
			PxTransform(_bodyLocalPos),
			_bodyPart,
			PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));

		if (!_joint) mSampleHumanoid->fatalError("PxDistanceJointCreate failed!");
		mJoints.push_back(_joint);
#ifndef ADD_JOINT
		_joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
		_joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
		if (p_swingLimit)
		{
			_joint->setPyramidSwingLimit(*p_swingLimit);
			delete p_swingLimit;
			p_swingLimit = 0;

			PxD6JointDrive _swingDrive;
			_swingDrive.stiffness = _swing1_param.stiffness; // Set the drive stiffness
			_swingDrive.damping = _swing1_param.damping;   // Set the drive damping
			_swingDrive.forceLimit = PX_MAX_F32; // Set the force limit (PX_MAX_F32 for no limit)
			_joint->setDrive(PxD6Drive::eSWING, _swingDrive);
			// Set motion for each axis
			_joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
			_joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
		}

		if (p_swing1Limit)
		{
			_joint->setSwingLimit(*p_swing1Limit);
			delete p_swing1Limit;
			p_swing1Limit = 0;

			PxD6JointDrive _swingDrive;
			_swingDrive.stiffness = _swing1_param.stiffness; // Set the drive stiffness
			_swingDrive.damping = _swing1_param.damping;   // Set the drive damping
			_swingDrive.forceLimit = PX_MAX_F32; // Set the force limit (PX_MAX_F32 for no limit)
			_joint->setDrive(PxD6Drive::eSWING, _swingDrive);
			_joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);

			/*
			_joint->setDrivePosition(PxTransform(PxVec3(
				0.0f,
				0.0f,
				0.0f)));
			*/
		}

		if (p_swing2Limit)
		{
			_joint->setSwingLimit(*p_swing2Limit);
			delete p_swing2Limit;
			p_swing2Limit = 0;

			PxD6JointDrive _swingDrive;
			_swingDrive.stiffness = _swing2_param.stiffness; // Set the drive stiffness
			_swingDrive.damping = _swing2_param.damping;   // Set the drive damping
			_swingDrive.forceLimit = PX_MAX_F32; // Set the force limit (PX_MAX_F32 for no limit)
			_joint->setDrive(PxD6Drive::eSWING, _swingDrive);

			_joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
		}

		if (p_twistLimit)
		{
			_joint->setTwistLimit(*p_twistLimit);
			delete p_twistLimit;
			p_twistLimit = 0;

			// Set up the drive for a specific degree of freedom, e.g., the twist
			PxD6JointDrive _twistDrive;
			_twistDrive.stiffness = _twist_param.stiffness; // Set the drive stiffness
			_twistDrive.damping = _twist_param.damping;   // Set the drive damping
			_twistDrive.forceLimit = PX_MAX_F32; // Set the force limit (PX_MAX_F32 for no limit)

			// Enable the drive for the twist degree of freedom
			_joint->setDrive(PxD6Drive::eTWIST, _twistDrive);
			_joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
		}
		else
		{
			_joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);
		}
		// Create joint ends
#endif


		this->ProcessMujoco(pNow->pChild, _bodyPart);
		pNow = pNow->pSibling;
	}


}




	
int Humanoid::loadMujoco()
{
	char buffer[MAX_PATH];
	if (GetModuleFileName(NULL, buffer, MAX_PATH) == 0) {
		std::cerr << "Error getting module file name." << std::endl;
		return 1;
	}

	std::string fullPath(buffer);
	std::string::size_type pos = fullPath.find_last_of("\\/");
	std::string directory;
	if (pos != std::string::npos)
	{
		directory = fullPath.substr(0, pos);
		std::cout << "Executable Directory: " << directory << std::endl;
	}
	else 
	{
		std::cerr << "Error finding directory name." << std::endl;
	}

	std::string filename = "\\amp_humanoid_torso.xml"; // Replace 'example.txt' with your desired file name
	std::string fullFilePath = directory + filename;

	MujocoXmlParser _parser;
	float _posRoot[] = { 0.0f, 0.0f, 0.0f };
	BodyParam* _bodyParam = _parser.Load(fullFilePath.c_str(), _posRoot, 5.0f);

	std::vector<PxTransform> _geomPoses;
	std::vector<const PxGeometry*> _geometries;
	physx::PxVec3 _bodyGlobalPos = physx::PxVec3(_posRoot[0], _posRoot[1], _posRoot[2]);
	PxRigidDynamic* _dynamicRoot = mSampleHumanoid->createCompound(_bodyGlobalPos, _geomPoses, _geometries, NULL, mMaterial, 1.0f)->is<PxRigidDynamic>();
	ProcessMujoco(_bodyParam, _dynamicRoot);
	_bodyParam->Destroy();
	_bodyParam = 0;


	return 0;

}

void Humanoid::createTorso(PxRigidDynamic* mainBody, PxVec3 localPos, PxRigidDynamic* motor, PxVec3 motorAttachmentPos)
{
	
	PxVec3 torsoPos = mainBody->getGlobalPose().p + localPos;

	std::vector<PxTransform> poses;
	std::vector<const PxGeometry*> geometries;
	
	PxVec3 startPos = torsoPos + PxVec3(0, 0, 0);
	/*
	PxRigidDynamic* spine = NULL;
	{
		PxTransform poseA = PxTransform(PxVec3(0, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 0, 1)));

		poses.clear(); 
		geometries.clear();

		poses.push_back(poseA);
		PxCapsuleGeometry capsGeomD = PxCapsuleGeometry(0.5f, 0.4f);
		geometries.push_back(&capsGeomD);
		spine = mSampleHumanoid->createCompound(startPos + PxVec3(-1.0f, 4.1f, -1.2f), poses, geometries, NULL, mMaterial, 1.0f)->is<PxRigidDynamic>();
		if (!spine) mSampleHumanoid->fatalError("createCompound failed!");
		mActors.push_back(spine);
	}*/




	/*
	
	PxDistanceJoint* jointHanger = NULL;
	{
		jointHanger = PxDistanceJointCreate(mSampleHumanoid->getPhysics(),
			mainBody,
			PxTransform(PxVec3(-1, 5, -1)),
			spine,
			PxTransform(PxVec3(0, -0.9, 0)));

		if (!jointHanger) mSampleHumanoid->fatalError("PxDistanceJointCreate failed!");
		mJoints.push_back(jointHanger);

		// set min & max distance to dist0
		float dist0 = 0.001;
		jointHanger->setMaxDistance(dist0);
		jointHanger->setMinDistance(dist0);
		// setup damping & spring
		jointHanger->setDamping(10000.0f);
		jointHanger->setStiffness(10000.0f);
		jointHanger->setDistanceJointFlags(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED | PxDistanceJointFlag::eMIN_DISTANCE_ENABLED);
		
		PxFilterData filterData1, filterData2;
#define CATEGORY_1 1
		// Assuming CATEGORY_1 is a predefined category for these types of objects
		filterData1.word0 = CATEGORY_1; // Category bits
		filterData2.word0 = CATEGORY_1; // Category bits

		// Set mask bits so that they don't collide with each other
		filterData1.word1 = ~CATEGORY_1; // Collides with everything but CATEGORY_1
		filterData2.word1 = ~CATEGORY_1; // Collides with everything but CATEGORY_1

		// Apply filter data to the shapes of each body
		PxShape* shape; mainBody->getShapes(&shape, 1);
		shape->setSimulationFilterData(filterData1);
		spine->getShapes(&shape, 1);
		shape->setSimulationFilterData(filterData2);
		

	}
	*/
	
	PxRigidDynamic* thighLeft = NULL;
	{
		PxTransform poseA = PxTransform(PxVec3(0, 0, 0), PxQuat(PxHalfPi*0.1f, PxVec3(0, 0, 1)));

		poses.clear();
		geometries.clear();

		poses.push_back(poseA);
		PxCapsuleGeometry capsGeomD = PxCapsuleGeometry(0.5f, 0.4f);
		geometries.push_back(&capsGeomD);
		thighLeft = mSampleHumanoid->createCompound(startPos + PxVec3(-0.1f, 2.3f, -0.1f), poses, geometries, NULL, mMaterial, 1.0f)->is<PxRigidDynamic>();
		if (!thighLeft) mSampleHumanoid->fatalError("createCompound failed!");
		mActors.push_back(thighLeft);
		physx::PxVec3 initialLinearVelocity(0.0f, -5.0f, 0.0f);
		thighLeft->setLinearVelocity(initialLinearVelocity);
	}
	
	{
		
#define CATEGORY_1 16
#define CATEGORY_2 256


		// Ensure that your filter shader/callback is set up to handle these flags
		// This setup is usually done when you create the PhysX scene

		PxFilterData filterData1;
	//	filterData1.word0 = CATEGORY_1;
	//	filterData1.word1 = CATEGORY_2;
	//	filterData1.word2 = CATEGORY_2;

		PxFilterData filterData2;
	//	filterData2.word0 = CATEGORY_2;
	//	filterData2.word1 = CATEGORY_1;
	//	filterData2.word2 = CATEGORY_2;
		// Assuming CATEGORY_1 is a predefined category for these types of objects


		// Apply filter data to the shapes of each body
		int numShapes = mainBody->getNbShapes();
		numShapes = thighLeft->getNbShapes();
		PxShape* shape; mainBody->getShapes(&shape, 1);
		shape->setSimulationFilterData(filterData1);
		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);

		thighLeft->getShapes(&shape, 1);
		shape->setSimulationFilterData(filterData2);

		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
		
		/*
		// Apply filter data to the shapes of each body
		int numShapes = mainBody->getNbShapes();
		numShapes = thighLeft->getNbShapes();
		PxShape* shape; mainBody->getShapes(&shape, 1);
		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);

		thighLeft->getShapes(&shape, 1);
		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
		*/

	}
	
	
	
	PxD6Joint* jointHipLeft = NULL;
	{
	/*

		jointHipLeft = PxD6JointCreate(mSampleHumanoid->getPhysics(),
			spine,
			PxTransform(PxVec3(0, 0.9, 0)),
			thighLeft,
			PxTransform(PxVec3(0, -0.9, 0)));

		if (!jointHipLeft) mSampleHumanoid->fatalError("PxDistanceJointCreate failed!");
		mJoints.push_back(jointHipLeft);

		
		// Set motion for each axis
		jointHipLeft->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
		jointHipLeft->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
		jointHipLeft->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
		
		
		// Set swing limits (for the cone)
		PxJointLimitPyramid swingLimit(-0.25f * PxPi, 0.25f * PxPi, -0.25f * PxPi, 0.25f * PxPi); // 45 degrees in radians
		jointHipLeft->setPyramidSwingLimit(swingLimit);

		// Set twist limits
		PxJointAngularLimitPair twistLimit(-PxPi / 4, PxPi / 4); // +/- 45 degrees in radians
		jointHipLeft->setTwistLimit(twistLimit);

		// Set up the drive for a specific degree of freedom, e.g., the twist
		PxD6JointDrive drive;
		drive.stiffness = 100.0f; // Set the drive stiffness
		drive.damping = 100.0f;   // Set the drive damping
		drive.forceLimit = PX_MAX_F32; // Set the force limit (PX_MAX_F32 for no limit)

		// Enable the drive for the twist degree of freedom
		jointHipLeft->setDrive(PxD6Drive::eTWIST, drive);
		jointHipLeft->setDrive(PxD6Drive::eSWING, drive);

		PxFilterData filterData1, filterData2;
#define CATEGORY_1 1
		// Assuming CATEGORY_1 is a predefined category for these types of objects
		filterData1.word0 = CATEGORY_1; // Category bits
		filterData2.word0 = CATEGORY_1; // Category bits

		// Set mask bits so that they don't collide with each other
		filterData1.word1 = ~CATEGORY_1; // Collides with everything but CATEGORY_1
		filterData2.word1 = ~CATEGORY_1; // Collides with everything but CATEGORY_1

		// Apply filter data to the shapes of each body
		PxShape* shape = NULL; 
		thighLeft->getShapes(&shape, 1);
		shape->setSimulationFilterData(filterData1);
		spine->getShapes(&shape, 1);
		shape->setSimulationFilterData(filterData2
		*/

	}

	
	/*
	PxRigidDynamic* calfLeft = NULL;
	{
		PxTransform poseA = PxTransform(PxVec3(0, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 0, 1)));

		poses.clear();
		geometries.clear();

		poses.push_back(poseA);
		geometries.push_back(&capsGeomD);
		calfLeft = mSampleHumanoid->createCompound(startPos, poses, geometries, NULL, mMaterial, density)->is<PxRigidDynamic>();
		if (!calfLeft) mSampleHumanoid->fatalError("createCompound failed!");
		mActors.push_back(calfLeft);
	}

	PxD6Joint* jointKneeLeft = NULL;
	{
		jointKneeLeft = PxD6JointCreate(mSampleHumanoid->getPhysics(),
			thighLeft,
			PxTransform(PxVec3(0, d, 0)),
			calfLeft,
			PxTransform(PxVec3(0, -d, 0)));

		if (!jointKneeLeft) mSampleHumanoid->fatalError("PxDistanceJointCreate failed!");
		mJoints.push_back(jointKneeLeft);

		// Set motion for each axis
		jointKneeLeft->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
		jointKneeLeft->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
		jointKneeLeft->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);

		// Set swing limits (for the cone)
		PxJointLimitPyramid swingLimit(-0.25f * PxPi, 0.25f * PxPi, -0.25f * PxPi, 0.25f * PxPi); // 45 degrees in radians
		jointKneeLeft->setPyramidSwingLimit(swingLimit);

		// Set twist limits
		PxJointAngularLimitPair twistLimit(-PxPi / 4, PxPi / 4); // +/- 45 degrees in radians
		jointKneeLeft->setTwistLimit(twistLimit);
		// Assuming d6Joint is a previously created PxD6Joint*

		// Set up the drive for a specific degree of freedom, e.g., the twist
		PxD6JointDrive drive;
		drive.stiffness = 1000.0f; // Set the drive stiffness
		drive.damping = 1000.0f;   // Set the drive damping
		drive.forceLimit = PX_MAX_F32; // Set the force limit (PX_MAX_F32 for no limit)

		// Enable the drive for the twist degree of freedom
		jointKneeLeft->setDrive(PxD6Drive::eTWIST, drive);
		jointKneeLeft->setDrive(PxD6Drive::eSWING, drive);
	}

	PxRigidDynamic* footLeft = NULL;
	{
		footLeft = mSampleHumanoid->createBox(startPos + PxVec3(0, 2, 0), boxGeomC.halfExtents, NULL, mMaterial, density, true)->is<PxRigidDynamic>();
		if (!footLeft) mSampleHumanoid->fatalError("createBox failed!");

		mActors.push_back(footLeft);
	}

	PxRevoluteJoint* jointAnkleLeft = NULL;
	{
		jointAnkleLeft = PxRevoluteJointCreate(mSampleHumanoid->getPhysics(),
			calfLeft,
			PxTransform(PxVec3(0, d, 0), PxQuat(0, PxVec3(0, 1, 0))), // -PxHalfPi
			footLeft, 
			PxTransform(PxVec3(0, -boxGeomC.halfExtents.y, 0), PxQuat(0, PxVec3(0, 1, 0))));

		if (!jointAnkleLeft) mSampleHumanoid->fatalError("PxRevoluteJointCreate failed!");
		mJoints.push_back(jointAnkleLeft);

		// Assume 'joint' is a previously created PxRevoluteJoint* instance

		// Enable limits
		jointAnkleLeft->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);

		// Create limit parameters: lower limit, upper limit, and restitution
		// Angles are in radians. For example, setting limits from -90 to 90 degrees:
		PxJointAngularLimitPair limits(-PxPi / 4, PxPi / 4, 0.01f); // 0.01 is a small restitution

		// Apply the limits to the joint
		jointAnkleLeft->setLimit(limits);
	}
	*/


}

void Humanoid::update(PxReal dt)
{
	PxSceneWriteLock scopedLock(mSampleHumanoid->getActiveScene());

	{
		// check if I have to be reset
		PxTransform pose = mHumanoidBody->getGlobalPose();
		PxVec3 upVect = PxVec3(0,1,0);
		PxVec3 crabUp = pose.rotate(upVect);
		PxReal angle = upVect.dot(crabUp);
		if(angle <= 0.1f)
		{
	//		mRespawnMe = true;
		}
	}

	PxReal maxVelocity = 16.0f;
	PxReal velDamping = 0.8f;

	if(mRunning == 0)
		flushAccelerationBuffer();

	// add up elapsed time
	mAccumTime += dt;

	// submit accum time to AI time before starting the PxTask
	if(mRunning == 0)
	{
		mElapsedTime = mAccumTime;
		mAccumTime = 0;
		mHumanoidPos = mSampleHumanoid->mHumanoidActor ? mSampleHumanoid->mHumanoidActor->getGlobalPose().p : PxVec3(0);

#if DEBUG_RENDERING
		// run immediately
		scanForObstacles();
		updateState();
#endif
	}
}

void Humanoid::run()
{
#if !DEBUG_RENDERING
	mRunning = 1;

	// run as a separate task/thread
	updateState();

	mRunning = 0;
#endif
}


void Humanoid::setAcceleration(PxReal leftAcc, PxReal rightAcc)
{
	mAccelerationBuffer[0] = -leftAcc;
	mAccelerationBuffer[1] = -rightAcc;
}


void Humanoid::flushAccelerationBuffer()
{
	mAcceleration[0] =  mAccelerationBuffer[0];
	mAcceleration[1] =  mAccelerationBuffer[1];
}

void Humanoid::initState(HumanoidState::Enum state)
{
	mHumanoidState = state;
	mStateTime = gDefaultStateTime[mHumanoidState];
}

void Humanoid::updateState()
{
	// update remaining time in current state
	// transition if needed
	mStateTime -= mElapsedTime;
	mElapsedTime = 0;
	if(mStateTime <= 0.0f)
	{
		initState(HumanoidState::eMOVE_FWD);
	}

	PxTransform crabPose;
	{
		PxSceneReadLock scopedLock(mSampleHumanoid->getActiveScene());
		crabPose = mHumanoidBody->getGlobalPose();
	}


	PxReal leftAcc = 0, rightAcc = 0;
	// compute fwd and bkwd distances
	static const PxReal minDist = 10.0f;
	static const PxReal fullSpeedDist = 50.0f;
	static const PxReal recipFullSpeedDist = 1.0f/fullSpeedDist;
	PxReal fDist = 0, bDist = 0;
	fDist = PxMin(mDistances[0], PxMin(mDistances[1], mDistances[2]));
	bDist = PxMin(mDistances[3], PxMin(mDistances[4], mDistances[5]));

	// handle states
	if(mHumanoidState == HumanoidState::eMOVE_FWD)
	{
		if(fDist < minDist)
		{
			initState(HumanoidState::eMOVE_BKWD);
		}
		else
		{
			leftAcc = PxMin(fullSpeedDist, mDistances[0])*recipFullSpeedDist*2.0f - 1.0f;
			rightAcc = PxMin(fullSpeedDist, mDistances[2])*recipFullSpeedDist*2.0f - 1.0f;
			leftAcc *= 3.0f;
			rightAcc *= 3.0f;
		}
	}
	else if (mHumanoidState == HumanoidState::eMOVE_BKWD)
	{
		if(bDist < minDist)
		{
			// find rotation dir, where we have some free space
			bool rotateLeft = mDistances[0] < mDistances[2];
			initState(rotateLeft ? HumanoidState::eROTATE_LEFT : HumanoidState::eROTATE_RIGHT);
		}
		else
		{
			leftAcc = -(PxMin(fullSpeedDist, mDistances[5])*recipFullSpeedDist*2.0f - 1.0f);
			rightAcc = -(PxMin(fullSpeedDist, mDistances[3])*recipFullSpeedDist*2.0f - 1.0f);
			leftAcc *= 3.0f;
			rightAcc *= 3.0f;
		}
	}
	else if (mHumanoidState == HumanoidState::eROTATE_LEFT)
	{
		leftAcc = -3.0f;
		rightAcc = 3.0f;
		if(fDist > minDist)
		{
			initState(HumanoidState::eMOVE_FWD);
		}
	}
	else if (mHumanoidState == HumanoidState::eROTATE_RIGHT)
	{
		leftAcc = 3.0f;
		rightAcc = -3.0f;
		if(fDist > minDist)
		{
			initState(HumanoidState::eMOVE_FWD);
		}
	}
	else if (mHumanoidState == HumanoidState::ePANIC)
	{
		if(mSampleHumanoid->mHumanoidActor)
		{

		}
	}
	else if (mHumanoidState == HumanoidState::eWAITING)
	{
		// have a break
	}

	// change acceleration
	setAcceleration(leftAcc, rightAcc);

#if DEBUG_RENDERING
	PxVec3 startPosL = crabPose.transform(PxVec3(0,2,-1));
	PxVec3 startPosR =  crabPose.transform(PxVec3(0,2,1));
	mSampleHumanoid->getDebugRenderer()->addLine(startPosL, startPosL + crabPose.q.rotate(PxVec3(1,0,0))*leftAcc, SampleRenderer::RendererColor(255,255,0));
	mSampleHumanoid->getDebugRenderer()->addLine(startPosR, startPosR + crabPose.q.rotate(PxVec3(1,0,0))*rightAcc, SampleRenderer::RendererColor(0,255,0));
#endif
}


