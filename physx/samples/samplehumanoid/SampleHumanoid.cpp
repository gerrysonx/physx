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

#include <thread>
#include <chrono>

#include "SampleHumanoid.h"
#include "SampleUtils.h"
#include "SampleAllocatorSDKClasses.h"
#include "RenderBaseActor.h"
#include "RendererMemoryMacros.h"
#include "RenderMaterial.h"

#include "PxTkFile.h"
#include "PxPhysicsAPI.h"
#include "extensions/PxExtensionsAPI.h"

#include <SamplePlatform.h>
#include <SampleUserInput.h>
#include <SampleUserInputIds.h>
#include "SampleHumanoidInputEventIds.h"
#include <SampleUserInputDefines.h>

#include "Humanoid.h"

#include "HumanoidCameraController.h"

// for loading the HeightField
#include "RenderMeshActor.h"
#include "PxTkBmpLoader.h"
//////////////////////////////

#include <algorithm>

using namespace PxToolkit;
using namespace SampleRenderer;
using namespace SampleFramework;

REGISTER_SAMPLE(SampleHumanoid, "SampleHumanoid")

static PxVec3				gBuoyancy = PxVec3(0, 1.0f, 0);
static PxRigidDynamic*		gTreasureActor = NULL;
static PxVec3				gForce = PxVec3(0, 0, 0);
static PxVec3				gTorque = PxVec3(0, 0, 0);
static const PxReal			gLinPower = 200.0f;
static const PxReal			gAngPower = 5000.0f;
static const PxReal			gHumanoidDensity = 3.0f;
static PxI32				gHumanoidHealth = 100;
static PxU32				gKeyFlags = 0;
static bool					gTreasureFound = false;
static bool					gResetScene = false;

static Humanoid*				gHumanoid = NULL;
static std::chrono::system_clock::time_point gTimeStartScene;
static bool gPhysicsActivated = false;
struct Movement
{
	enum Enum
	{
		eHUMANOID_FWD				= (1 << 0),
		eHUMANOID_BCKWD				= (1 << 1),
		eHUMANOID_ROTATE_LEFT		= (1 << 2),
		eHUMANOID_ROTATE_RIGHT		= (1 << 3),
		eSUBMAINE_FWD			= (1 << 4),
		eSUBMAINE_BCKWD			= (1 << 5),
		eSUBMAINE_UP			= (1 << 6),
		eSUBMAINE_DOWN			= (1 << 7),
	};
};


enum MaterialID
{
	MATERIAL_TERRAIN_MUD	 = 1000,
};

///////////////////////////////////////////////////////////////////////////////


SampleHumanoid::SampleHumanoid(PhysXSampleApplication& app)
	: PhysXSample(app)
	, mHumanoidActor(NULL)
	, mCameraAttachedToActor(NULL)
	, mHumanoidCameraController(NULL)
{
	mCreateGroundPlane	= false;
	//mStepperType = FIXED_STEPPER;
}

SampleHumanoid::~SampleHumanoid()
{
}

///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::customizeSample(SampleSetup& setup)
{
	setup.mName	= "SampleHumanoid";
}

///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::onInit()
{
	mNbThreads = PxMax(PxI32(shdfnd::Thread::getNbPhysicalCores())-1, 0);

	mCreateCudaCtxManager = true;

	PhysXSample::onInit();

	PxSceneWriteLock scopedLock(*mScene);

	mHumanoidCameraController = SAMPLE_NEW(HumanoidCameraController)();
	setCameraController(mHumanoidCameraController);

	mApplication.setMouseCursorHiding(true);
	mApplication.setMouseCursorRecentering(true);
	mHumanoidCameraController->init(PxTransform(PxIdentity));
	mHumanoidCameraController->setMouseSensitivity(0.5f);
	mHumanoidCameraController->setMouseLookOnMouseButton(false);

	getRenderer()->setAmbientColor(RendererColor(60, 60, 60));

	PxMaterial& material = getDefaultMaterial();
	material.setRestitution(0);
	material.setDynamicFriction(500.0f);
	material.setStaticFriction(500.0f);

	// set gravity
	getActiveScene().setGravity(PxVec3(0, -10.0f, 0));

	createMaterials();

	getRenderer()->setFog(SampleRenderer::RendererColor(16,16,40), 125.0f);

	PxRigidActor* heightField = loadTerrain("submarine_heightmap.bmp", 0.4f, 3.0f, 3.0f);
	if (!heightField) fatalError("Sample can not load file submarine_heightmap.bmp\n");

	// create ceiling plane
	PxReal d = 60.0f;
	PxTransform pose = PxTransform(PxVec3(0.0f, d, 0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, -1.0f)));
	PxRigidStatic* plane = getPhysics().createRigidStatic(pose);
	if(!plane) fatalError("createRigidStatic failed!");
	PxShape* shape = PxRigidActorExt::createExclusiveShape(*plane, PxPlaneGeometry(), material);
	if(!shape) fatalError("createShape failed!");
	getActiveScene().addActor(*plane);

	resetScene();
}

void SampleHumanoid::createMaterials()
{
	RAWTexture data;
	data.mName = "rock_diffuse2.dds";
	RenderTexture* gravelTexture = createRenderTextureFromRawTexture(data);

	RenderMaterial* terrainMaterial = SAMPLE_NEW(RenderMaterial)(*getRenderer(), PxVec3(0.5f, 0.25f, 0.125f), 1.0f, false, MATERIAL_TERRAIN_MUD, gravelTexture);
	mRenderMaterials.push_back(terrainMaterial);

}


PxRigidActor* SampleHumanoid::loadTerrain(const char* name, const PxReal heightScale, const PxReal rowScale, const PxReal columnScale) 
{
	PxRigidActor* heightFieldActor = NULL;
	BmpLoader loader;
	if(loader.loadBmp(getSampleMediaFilename(name))) 
	{
		PxU16 nbColumns = PxU16(loader.mWidth), nbRows = PxU16(loader.mHeight);
		PxHeightFieldDesc heightFieldDesc;
		heightFieldDesc.nbColumns = nbColumns;
		heightFieldDesc.nbRows = nbRows;
		PxU32* samplesData = (PxU32*)SAMPLE_ALLOC(sizeof(PxU32)*nbColumns * nbRows);
		heightFieldDesc.samples.data = samplesData;
		heightFieldDesc.samples.stride = sizeof(PxU32);
		PxU8* currentByte = (PxU8*)heightFieldDesc.samples.data;
		PxU8* loader_ptr = loader.mRGB;
		PxVec3Alloc* vertexesA = SAMPLE_NEW(PxVec3Alloc)[nbRows * nbColumns];
		PxF32* uvs = (PxF32*)SAMPLE_ALLOC(sizeof(PxF32) * nbRows * nbColumns * 2);
		PxVec3* vertexes = vertexesA;
		for (PxU32 row = 0; row < nbRows; row++) 
		{
			for (PxU32 column = 0; column < nbColumns; column++) 
			{
				PxHeightFieldSample* currentSample = (PxHeightFieldSample*)currentByte;
				currentSample->height = *loader_ptr;
				vertexes[row * nbColumns + column] = PxVec3(PxReal(row)*rowScale, 
					PxReal(currentSample->height * heightScale), 
					PxReal(column)*columnScale);

				uvs[(row * nbColumns + column)*2 + 0] = (float)column/7.0f;
				uvs[(row * nbColumns + column)*2 + 1] = (float)row/7.0f;

				currentSample->materialIndex0 = 0;
				currentSample->materialIndex1 = 0;
				currentSample->clearTessFlag();
				currentByte += heightFieldDesc.samples.stride;
				loader_ptr += 3 * sizeof(PxU8);
			}
		}
		PxHeightField* heightField = getCooking().createHeightField(heightFieldDesc, getPhysics().getPhysicsInsertionCallback());
		if(!heightField) fatalError("createHeightField failed!");
		// create shape for heightfield		
		PxTransform pose(PxVec3(-((PxReal)nbRows*rowScale) / 2.0f, 
			-20.0f, 
			-((PxReal)nbColumns*columnScale) / 2.0f), 
			PxQuat(PxIdentity));
		heightFieldActor = getPhysics().createRigidStatic(pose);
		if(!heightFieldActor) fatalError("createRigidStatic failed!");
		PxShape* shape = PxRigidActorExt::createExclusiveShape(*heightFieldActor, PxHeightFieldGeometry(heightField, PxMeshGeometryFlags(), heightScale, rowScale, columnScale), getDefaultMaterial());
		if(!shape) fatalError("createShape failed!");

		// add actor to the scene
		PxFilterData filterData1;
		// Assuming CATEGORY_1 is a predefined category for these types of objects
		filterData1.word2 = 0xffcf; // Category bits
		shape->setSimulationFilterData(filterData1);

		getActiveScene().addActor(*heightFieldActor);
		// create indices
		PxU32* indices = (PxU32*)SAMPLE_ALLOC(sizeof(PxU32)*((nbColumns - 1) * (nbRows - 1) * 3 * 2));
		for(int i = 0; i < (nbColumns - 1); ++i) 
		{
			for(int j = 0; j < (nbRows - 1); ++j) 
			{
				// first triangle
				indices[6 * (i * (nbRows - 1) + j) + 0] = (i + 1) * nbRows + j; 
				indices[6 * (i * (nbRows - 1) + j) + 1] = i * nbRows + j;
				indices[6 * (i * (nbRows - 1) + j) + 2] = i * nbRows + j + 1;
				// second triangle
				indices[6 * (i * (nbRows - 1) + j) + 3] = (i + 1) * nbRows + j + 1;
				indices[6 * (i * (nbRows - 1) + j) + 4] = (i + 1) * nbRows + j;
				indices[6 * (i * (nbRows - 1) + j) + 5] = i * nbRows + j + 1;
			}
		}
		// add mesh to renderer
		RAWMesh data;
		data.mName = name;
		data.mTransform = PxTransform(PxIdentity);
		data.mNbVerts = nbColumns * nbRows;
		data.mVerts = vertexes;
		data.mVertexNormals = NULL;
		data.mUVs = uvs;
		data.mMaterialID = MATERIAL_TERRAIN_MUD;
		data.mNbFaces = (nbColumns - 1) * (nbRows - 1) * 2;
		data.mIndices = indices;

		RenderMeshActor* hf_mesh = createRenderMeshFromRawMesh(data);
		if(!hf_mesh) fatalError("createRenderMeshFromRawMesh failed!");
		hf_mesh->setPhysicsShape(shape, heightFieldActor);
		shape->setFlag(PxShapeFlag::eVISUALIZATION, false);
		SAMPLE_FREE(indices);
		SAMPLE_FREE(uvs);
		DELETEARRAY(vertexesA);
		SAMPLE_FREE(samplesData);
	}

	return heightFieldActor;
}

///////////////////////////////////////////////////////////////////////////////

PxRigidDynamic* SampleHumanoid::createHumanoid(const PxVec3& inPosition, const PxReal yRot)
{
	PX_ASSERT(mHumanoidActor == NULL);

	std::vector<PxTransform> localPoses;
	std::vector<const PxGeometry*> geometries;

	// cabin
	PxSphereGeometry cabinGeom(1.5f);
	PxTransform	cabinPose = PxTransform(PxIdentity); 
	cabinPose.p.x = -0.5f;

	// engine
	PxBoxGeometry engineGeom(0.25f, 1.0f, 1.0f);
	PxTransform	enginePose = PxTransform(PxIdentity); 
	enginePose.p.x = cabinPose.p.x + cabinGeom.radius + engineGeom.halfExtents.x;

	// tanks
	PxCapsuleGeometry tankGeom(0.5f, 1.8f);
	PxTransform	tank1Pose = PxTransform(PxIdentity); 
	tank1Pose.p = PxVec3(0,-cabinGeom.radius, cabinGeom.radius);
	PxTransform	tank2Pose = PxTransform(PxIdentity); 
	tank2Pose.p = PxVec3(0,-cabinGeom.radius, -cabinGeom.radius);

	localPoses.push_back(cabinPose);
	geometries.push_back(&cabinGeom);
	localPoses.push_back(enginePose);
	geometries.push_back(&engineGeom);
	localPoses.push_back(tank1Pose);
	geometries.push_back(&tankGeom);
	localPoses.push_back(tank2Pose);
	geometries.push_back(&tankGeom);

	// put the shapes together into one actor
	mHumanoidActor = PhysXSample::createCompound(inPosition, localPoses, geometries, 0, mManagedMaterials[MATERIAL_YELLOW], gHumanoidDensity)->is<PxRigidDynamic>();
	
	if(!mHumanoidActor) fatalError("createCompound failed!");

	//disable the current and buoyancy effect for the sub.
	mHumanoidActor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);

	// set the filtering group for the humanoid

	mHumanoidActor->setLinearDamping(0.15f);
	mHumanoidActor->setAngularDamping(15.0f);

	PxTransform globalPose; 
	globalPose.p = inPosition;
	globalPose.q = PxQuat(yRot, PxVec3(0,1,0));
	mHumanoidActor->setGlobalPose(globalPose);

	mHumanoidActor->setCMassLocalPose(PxTransform(PxIdentity));

	return mHumanoidActor;
}


///////////////////////////////////////////////////////////////////////////////
static const char* getPlatformName()
{
#if PX_X86
	return "PC32";
#elif PX_X64
	return "PC64";
#elif PX_ARM_FAMILY
	return "ARM";
#else
	return "";
#endif
}

void SampleHumanoid::createDynamicActors()
{
	gHumanoid = SAMPLE_NEW(Humanoid)(*this, PxVec3(0, 30, 0), mManagedMaterials[MATERIAL_RED]);
	mHumanoids.push_back(gHumanoid);

	/*
	
	char theHumanoidName[256];
	sprintf(theHumanoidName, "human_%s.bin", getPlatformName());
	// If we have already had humanoid copy, just load it, or will create humanoid and export it
	char thePathBuffer[1024];
	const char* theHumanoidPath = getSampleOutputDirManager().getFilePath( theHumanoidName, thePathBuffer, false );
	SampleFramework::File* fp = NULL;
	PxToolkit::fopen_s(&fp, theHumanoidPath, "r" );
	if( fp )
	{
		shdfnd::printFormatted("loading the humanoid from file status: \n");
		gHumanoid = SAMPLE_NEW(Humanoid)(*this, theHumanoidPath, mManagedMaterials[MATERIAL_RED]);
		if (gHumanoid && !gHumanoid->getHumanoidBody())
		{
			delete gHumanoid;
			gHumanoid = NULL;
		}
		shdfnd::printFormatted(gHumanoid ? "successful\n":"failed\n");						
		fclose (fp); 
	}

	if( !gHumanoid )
	{
		gHumanoid = SAMPLE_NEW(Humanoid)(*this, PxVec3(0, 50, 0), mManagedMaterials[MATERIAL_RED]);
		shdfnd::printFormatted("humanoid file not found ... exporting humanoid file\n");
		gHumanoid->save(theHumanoidPath);
	}
	
	PX_ASSERT( gHumanoid );
	mHumanoids.push_back(gHumanoid);
	*/


}

///////////////////////////////////////////////////////////////////////////////

// Function to be executed by the thread
void ActivatePhysics() {
	std::cout << "Hello from the thread!" << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(3));
	for (PxRigidDynamic* actor : gHumanoid->mActors)
	{
		actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
	}
	std::cout << "Hello after work done from the thread!" << std::endl;
}

void SampleHumanoid::resetScene()
{
	gResetScene = false;
	const size_t nbHumanoids = mHumanoids.size();
	for(PxU32 i=0;i<nbHumanoids;i++)
	{
		delete mHumanoids[i];
	}
	mHumanoids.clear();

	const size_t nbJoints = mJoints.size();
	for(PxU32 i=0;i<nbJoints;i++)
		mJoints[i]->release();
	mJoints.clear();

	if(mHumanoidActor)
	{
		removeActor(mHumanoidActor);
		mHumanoidActor = NULL;
	}
	gHumanoidHealth = 100;

	if (gTreasureActor)
		removeActor(gTreasureActor);
	gTreasureActor = NULL;
	gTreasureFound = false;

	while(mPhysicsActors.size())
	{
		removeActor(mPhysicsActors[0]);
	}

	freeDeletedActors();

	createDynamicActors();

	// init camera orientation
	mHumanoidCameraController->init(PxVec3(-90, 60, 50), PxVec3(0.28f, 2.05f, 0.0f));
	mHumanoidCameraController->setMouseSensitivity(0.5f);
	mHumanoidCameraController->setFollowingMode(true);
	if (gHumanoid && gHumanoid->getHumanoidBody()) {
		mCameraAttachedToActor = gHumanoid->getHumanoidBody();
	}


	gTimeStartScene = std::chrono::system_clock::now();
	gPhysicsActivated = false;
	
}

///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::onShutdown()
{
	{
		PxSceneWriteLock scopedLock(*mScene);

		const size_t nbHumanoids = mHumanoids.size();
		for(PxU32 i=0;i<nbHumanoids;i++)
			delete mHumanoids[i];
		mHumanoids.clear();

		// free crabs' memory
		const size_t nbDelHumanoidsMem = mHumanoidsMemoryDeleteList.size();
		for(PxU32 i = 0; i < nbDelHumanoidsMem; i++)
			SAMPLE_FREE(mHumanoidsMemoryDeleteList[i]);
		mHumanoidsMemoryDeleteList.clear();

		gHumanoid = NULL;

		const size_t nbJoints = mJoints.size();
		for(PxU32 i=0;i<nbJoints;i++)
			mJoints[i]->release();
		mJoints.clear();

		gTreasureActor = NULL;

		DELETESINGLE(mHumanoidCameraController);
	}
	
	PhysXSample::onShutdown();
}

///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::onSubstep(float dtime)
{
	// user input -> forces
	handleInput();

	// delay free crabs' memory
	const size_t nbDelHumanoidsMem = mHumanoidsMemoryDeleteList.size();
	for(PxU32 i = 0; i < nbDelHumanoidsMem; i++)
		SAMPLE_FREE(mHumanoidsMemoryDeleteList[i]);
	mHumanoidsMemoryDeleteList.clear();

	// change current every 0.01s
	static PxReal sElapsedTime = 0.0f;
	sElapsedTime += mPause ? 0 : dtime;
	if(sElapsedTime > 0.01f)
	{
		static PxReal angle = 0;
		angle += sElapsedTime*0.01f;
		angle = angle < (PxTwoPi) ? angle : angle - PxTwoPi;
		sElapsedTime = 0;
		
		gBuoyancy.z = 0.15f * PxSin(angle * 50);
		PxQuat yRot = PxQuat(angle, PxVec3(0,1,0));
		gBuoyancy = yRot.rotate(gBuoyancy);

		// apply external forces to seamines
		PxSceneWriteLock scopedLock(*mScene);
	}

	if(mHumanoidActor)
	{
		PxSceneWriteLock scopedLock(*mScene);

		//convert forces from humanoid the humanoid's body local space to global space
		PxQuat humanoidOrientation = mHumanoidActor->getGlobalPose().q;
		gForce = humanoidOrientation.rotate(gForce);
		gTorque = humanoidOrientation.rotate(gTorque);

		// add also current forces to humanoid
		gForce.z += gBuoyancy.z * 5.0f;

		// apply forces in global space and reset
		mHumanoidActor->addForce(gForce);
		mHumanoidActor->addTorque(gTorque);
		gForce = PxVec3(0);
		gTorque = PxVec3(0);
	}
#ifdef ASYNC_OP
	if (!gPhysicsActivated)
	{
		// Get the second time point
		auto end = std::chrono::system_clock::now();

		// Calculate the difference between the two time points
		auto elapsed = end - gTimeStartScene;

		// Convert the duration to seconds
		auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

		std::cout << "Time difference in seconds: " << elapsedSeconds << std::endl;
		if (elapsedSeconds > 300)
		{
			ActivatePhysics();
			gPhysicsActivated = true;
		}

	}
#endif

}

void SampleHumanoid::onSubstepSetup(float dt, PxBaseTask* completionTask)
{
	// set Humanoids continuation to ensure the completion task
	// is not run before Humanoid update has completed
	const size_t nbHumanoids = mHumanoids.size();
	for(PxU32 i = 0; i < nbHumanoids; i++)
	{
		Humanoid* humanoid = mHumanoids[i];
		humanoid->update(dt);
		humanoid->setContinuation(completionTask);
	}
}

void SampleHumanoid::onSubstepStart(float dtime)
{
	// kick off humanoid updates in parallel to simulate
	const size_t nbHumanoids = mHumanoids.size();
	for(PxU32 i = 0; i < nbHumanoids; i++)
	{
		Humanoid* humanoid = mHumanoids[i];
		// inverted stepper: skip humanoid updates right after creation, 
		// humanoid task is not in the pipeline at this point (onSubstepSetup not yet called).
		if(humanoid->getTaskManager() == NULL)
			continue;
		humanoid->removeReference();
	}
}

void SampleHumanoid::onTickPreRender(float dtime)
{
	mScene->lockWrite();

	if(gResetScene)
		resetScene();

	// respawn Humanoids
	const size_t nbHumanoids = mHumanoids.size();
	for(PxU32 i = 0; i < nbHumanoids; i++)
	{
		Humanoid* humanoid = mHumanoids[i];
		if(humanoid->needsRespawn())
		{
			PxRigidDynamic* prevBody = humanoid->getHumanoidBody();
			PxVec3 prevPos = prevBody->getGlobalPose().p;
			delete humanoid;
			mHumanoids[i] = SAMPLE_NEW(Humanoid)(*this, prevPos, mManagedMaterials[MATERIAL_RED]);
			if(gHumanoid == humanoid)
				gHumanoid = mHumanoids[i];
			if(mCameraAttachedToActor == prevBody)
				mCameraAttachedToActor = mHumanoids[i]->getHumanoidBody();
		}
	}

	// update camera
	if(mCameraAttachedToActor)
		mHumanoidCameraController->updateFollowingMode(getCamera(), dtime, mCameraAttachedToActor->getGlobalPose().p);

	mScene->unlockWrite();

	// start the simulation
	PhysXSample::onTickPreRender(dtime);
}

///////////////////////////////////////////////////////////////////////////////

PxFilterFlags SampleHumanoidFilterShader(	
	PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	// let triggers through
	if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}
	// generate contacts for all that were not filtered above
	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	// trigger the contact callback for pairs (A,B) where 
	// the filtermask of A contains the ID of B and vice versa.
	int _shift_filter_data_0 = filterData0.word2 << 1;
	int _shift_filter_data_1 = filterData1.word2 << 1;
	if (filterData0.word2 * filterData0.word2 != 0 && (_shift_filter_data_0 == filterData1.word2 || filterData0.word2 == _shift_filter_data_1))
	{
	//	pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
	//	return PxFilterFlag::eKILL;
	}

	if (filterData0.word2 != 0xffcf && filterData1.word2 != 0xffcf)
	{
		return PxFilterFlag::eKILL;
	}
	else
	{
		return PxFilterFlag::eDEFAULT;
	}

//	return PxFilterFlag::eKILL;
	
//	return PxFilterFlag::eDEFAULT;
}

void SampleHumanoid::onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
{
	for (PxU32 i = 0; i < nbPairs; i++)
	{
		const PxContactPair& cp = pairs[i];

		if (cp.events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
		{
			PxActor* oneActor = pairHeader.actors[0];
			PxActor* otherActor = pairHeader.actors[1];
		}
	}
}
///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::customizeSceneDesc(PxSceneDesc& sceneDesc)
{
	sceneDesc.filterShader = SampleHumanoidFilterShader;
	sceneDesc.simulationEventCallback	= this;
	sceneDesc.flags						|= PxSceneFlag::eREQUIRE_RW_LOCK;
}

///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::onTrigger(PxTriggerPair* pairs, PxU32 count)
{
	for(PxU32 i=0; i < count; i++)
	{
		// ignore pairs when shapes have been deleted
		if (pairs[i].flags & (PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER | PxTriggerPairFlag::eREMOVED_SHAPE_OTHER))
			continue;

		if((pairs[i].otherActor == mHumanoidActor) && (pairs[i].triggerActor == gTreasureActor))
		{
			gTreasureFound = true;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::helpRender(PxU32 x, PxU32 y, PxU8 textAlpha)
{
	SampleRenderer::Renderer* renderer = getRenderer();
	const PxU32 yInc = 18;
	const PxReal scale = 0.5f;
	const PxReal shadowOffset = 6.0f;
	const RendererColor textColor(255, 255, 255, textAlpha);
	const bool isMouseSupported = getApplication().getPlatform()->getSampleUserInput()->mouseSupported();
	const bool isPadSupported = getApplication().getPlatform()->getSampleUserInput()->gamepadSupported();
	const char* msg;

	msg = mApplication.inputInfoMsg("Press "," to toggle various debug visualization", TOGGLE_VISUALIZATION, -1);
	if(msg)
		renderer->print(x, y += yInc, msg,scale, shadowOffset, textColor);
	msg = mApplication.inputInfoMsg("Press "," to restart", SCENE_RESET,-1);
	if(msg)
		renderer->print(x, y += yInc, msg,scale, shadowOffset, textColor);
	if (isMouseSupported && isPadSupported)
		renderer->print(x, y += yInc, "Use mouse or right stick to rotate the camera", scale, shadowOffset, textColor);
	else if (isMouseSupported)
		renderer->print(x, y += yInc, "Use mouse to rotate the camera", scale, shadowOffset, textColor);
	else if (isPadSupported)
		renderer->print(x, y += yInc, "Use right stick to rotate the camera", scale, shadowOffset, textColor);
	msg = mApplication.inputInfoMsg("Press "," to switch between humanoid/humanoid/flyCam", CAMERA_SWITCH, -1);
	if(msg)
		renderer->print(x, y += yInc, msg,scale, shadowOffset, textColor);

	if(mCameraAttachedToActor == mHumanoidActor)
	{
		renderer->print(x, y += yInc, "Humanoid Controller:", scale, shadowOffset, textColor);
		const char* msg = mApplication.inputInfoMsg("Press "," to move along view direction", HUMANOID_FORWARD, HUMANOID_BACKWARD);
		if(msg)
			renderer->print(x, y += yInc, msg,scale, shadowOffset, textColor);
		msg = mApplication.inputInfoMsg("Press "," to raise and dive", HUMANOID_UP, HUMANOID_DOWN);
		if(msg)
			renderer->print(x, y += yInc, msg,scale, shadowOffset, textColor);
	}
	else if(gHumanoid && (mCameraAttachedToActor == gHumanoid->getHumanoidBody()))
	{
		renderer->print(x, y += yInc, "Humanoid Controller:", scale, shadowOffset, textColor);
	}
	else
	{
		renderer->print(x, y += yInc, "Fly Cam Controller:", scale, shadowOffset, textColor);
		if (isPadSupported)
			renderer->print(x, y += yInc, "Use left stick to move",scale, shadowOffset, textColor);
		const char* msg = mApplication.inputMoveInfoMsg("Press ","  to move", CAMERA_MOVE_FORWARD,CAMERA_MOVE_BACKWARD, CAMERA_MOVE_LEFT, CAMERA_MOVE_RIGHT);
		if(msg)
			renderer->print(x, y += yInc, msg,scale, shadowOffset, textColor);
		msg = mApplication.inputInfoMsg("Press "," to move fast", CAMERA_SHIFT_SPEED, -1);
		if(msg)
			renderer->print(x, y += yInc, msg, scale, shadowOffset, textColor);
	}
}

void SampleHumanoid::descriptionRender(PxU32 x, PxU32 y, PxU8 textAlpha)
{
	bool print=(textAlpha!=0.0f);

	if(print)
	{
		Renderer* renderer = getRenderer();
		const PxU32 yInc = 24;
		const PxReal scale = 0.5f;
		const PxReal shadowOffset = 6.0f;
		const RendererColor textColor(255, 255, 255, textAlpha);

		char line0[256]="This sample demonstrates the creation of jointed systems. In particular,";
		char line1[256]="a complex system of driven joints is introduced to model humanoid motion,";
		char line2[256]="while distance joints are used to model the tethering of exploding sea"; 
		char line3[256]="mines to the seabed.  Trigger shapes are presented to detect the";
		char line4[256]="arrival of the humanoid at a treasure chest on the ocean floor.";
		char line5[256]="Similarly, contact notification is used to report the overlap of the";
		char line6[256]="humanoid with the exploding sea mines.  Humanoid logic is governed by sdk";   
		char line7[256]="raycast results, and illustrates the application of the PhysX SDK task";
		char line8[256]="scheduler.";

		renderer->print(x, y+=yInc, line0, scale, shadowOffset, textColor);
		renderer->print(x, y+=yInc, line1, scale, shadowOffset, textColor);
		renderer->print(x, y+=yInc, line2, scale, shadowOffset, textColor);
		renderer->print(x, y+=yInc, line3, scale, shadowOffset, textColor);
		renderer->print(x, y+=yInc, line4, scale, shadowOffset, textColor);
		renderer->print(x, y+=yInc, line5, scale, shadowOffset, textColor);
		renderer->print(x, y+=yInc, line6, scale, shadowOffset, textColor);
		renderer->print(x, y+=yInc, line7, scale, shadowOffset, textColor);
		renderer->print(x, y+=yInc, line8, scale, shadowOffset, textColor);
	}
}

///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::customizeRender()
{
	SampleRenderer::Renderer* renderer = getRenderer();
	const PxU32 yInc = 18;
	const RendererColor textColor(255, 255, 255, 255);

	PxU32 width, height;
	renderer->getWindowSize(width, height);
	char healthBar[20];
	PxU32 h = gHumanoidHealth;
	sprintf(healthBar, "Health:%c%c%c%c%c%c%c%c%c%c", (h>90?'I':' '), (h>80?'I':' '), (h>70?'I':' '),(h>60?'I':' '),(h>50?'I':' '), 
	(h>40?'I':' '), (h>30?'I':' '), (h>20?'I':' '),(h>10?'I':' '),(h>0?'I':' '));
	renderer->print(width-130, height-yInc, healthBar);	
	if(gTreasureFound)
		renderer->print(width-160, height-2*yInc, "Treasure Found!");	
}

///////////////////////////////////////////////////////////////////////////////

static void setFlag(PxU32& flags, PxU32 flag, bool set)
{
	if(set)
		flags |= flag;
	else
		flags &= ~flag;
}

///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::onPointerInputEvent(const SampleFramework::InputEvent& ie, physx::PxU32 x, physx::PxU32 y, physx::PxReal dx, physx::PxReal dy, bool val)
{
	if(ie.m_Id== CAMERA_MOUSE_LOOK || !mHumanoidActor || mCameraAttachedToActor != mHumanoidActor)
		PhysXSample::onPointerInputEvent(ie,x,y,dx,dy, val);

	switch (ie.m_Id)
	{
	case HUMANOID_FORWARD:
		{
			setFlag(gKeyFlags, Movement::eSUBMAINE_FWD, val);
		}
		break;
	case HUMANOID_BACKWARD:
		{
			setFlag(gKeyFlags, Movement::eSUBMAINE_BCKWD, val);
		}
		break;
	default:
		break;
	}		
}

//////////////////////////////////////////////////////////////////////////

void SampleHumanoid::collectInputEvents(std::vector<const SampleFramework::InputEvent*>& inputEvents)
{
	PhysXSample::collectInputEvents(inputEvents);

	getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(CAMERA_MOVE_UP);
	getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(CAMERA_MOVE_DOWN);
	getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(CAMERA_SPEED_INCREASE);
	getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(CAMERA_SPEED_DECREASE);
	getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(CAMERA_MOVE_BUTTON);
	getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(MOUSE_LOOK_BUTTON);
	getApplication().getPlatform()->getSampleUserInput()->unregisterInputEvent(SPAWN_DEBUG_OBJECT);

	//digital mouse events
	DIGITAL_INPUT_EVENT_DEF(HUMANOID_FORWARD,											MOUSE_BUTTON_LEFT,			MOUSE_BUTTON_LEFT,			MOUSE_BUTTON_LEFT	);
	DIGITAL_INPUT_EVENT_DEF(HUMANOID_BACKWARD,											MOUSE_BUTTON_RIGHT,			MOUSE_BUTTON_RIGHT,			MOUSE_BUTTON_RIGHT	);
										
	//digital keyboard events										
	DIGITAL_INPUT_EVENT_DEF(HUMANOID_FORWARD, SCAN_CODE_FORWARD, SCAN_CODE_FORWARD, SCAN_CODE_FORWARD);
	DIGITAL_INPUT_EVENT_DEF(HUMANOID_BACKWARD, SCAN_CODE_BACKWARD, SCAN_CODE_BACKWARD, SCAN_CODE_BACKWARD);
	DIGITAL_INPUT_EVENT_DEF(HUMANOID_LEFT, SCAN_CODE_LEFT, SCAN_CODE_LEFT, SCAN_CODE_LEFT);
	DIGITAL_INPUT_EVENT_DEF(HUMANOID_RIGHT, SCAN_CODE_RIGHT, SCAN_CODE_RIGHT, SCAN_CODE_RIGHT);

	DIGITAL_INPUT_EVENT_DEF(CAMERA_SWITCH,												SCAN_CODE_DOWN,				SCAN_CODE_DOWN,				SCAN_CODE_DOWN		);
	DIGITAL_INPUT_EVENT_DEF(SCENE_RESET,												WKEY_R,						OSXKEY_R,					LINUXKEY_R			);
							
	//digital gamepad events							
	DIGITAL_INPUT_EVENT_DEF(HUMANOID_FORWARD,											GAMEPAD_RIGHT_SHOULDER_BOT,	GAMEPAD_RIGHT_SHOULDER_BOT,	LINUXKEY_UNKNOWN	);
	DIGITAL_INPUT_EVENT_DEF(HUMANOID_BACKWARD,											GAMEPAD_LEFT_SHOULDER_BOT,	GAMEPAD_LEFT_SHOULDER_BOT,	LINUXKEY_UNKNOWN	);
	DIGITAL_INPUT_EVENT_DEF(HUMANOID_UP,												GAMEPAD_RIGHT_SHOULDER_TOP,	GAMEPAD_RIGHT_SHOULDER_TOP,	LINUXKEY_UNKNOWN	);
	DIGITAL_INPUT_EVENT_DEF(HUMANOID_DOWN,												GAMEPAD_LEFT_SHOULDER_TOP,	GAMEPAD_LEFT_SHOULDER_TOP,	LINUXKEY_UNKNOWN	);
	DIGITAL_INPUT_EVENT_DEF(CAMERA_SWITCH,												GAMEPAD_RIGHT_STICK,		GAMEPAD_RIGHT_STICK,		LINUXKEY_UNKNOWN	);
	DIGITAL_INPUT_EVENT_DEF(SCENE_RESET,												GAMEPAD_LEFT_STICK,			GAMEPAD_LEFT_STICK,			LINUXKEY_UNKNOWN	);
		
	// analog gamepad events	
	ANALOG_INPUT_EVENT_DEF(HUMANOID_FORWARD_BACKWARD, GAMEPAD_DEFAULT_SENSITIVITY,		GAMEPAD_LEFT_STICK_Y,		OSXKEY_UNKNOWN,         	LINUXKEY_UNKNOWN	);
}

///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::onDigitalInputEvent(const SampleFramework::InputEvent& ie, bool val)
{
	if(mHumanoidActor && mCameraAttachedToActor == mHumanoidActor)
	{
		switch (ie.m_Id)
		{
		case HUMANOID_FORWARD:
			setFlag(gKeyFlags, Movement::eSUBMAINE_FWD, val);
			break;
		case HUMANOID_BACKWARD:
			setFlag(gKeyFlags, Movement::eSUBMAINE_BCKWD, val);
			break;
		case HUMANOID_UP:
			setFlag(gKeyFlags, Movement::eSUBMAINE_UP, val);
			break;
		case HUMANOID_DOWN:
			setFlag(gKeyFlags, Movement::eSUBMAINE_DOWN, val);
			break;
		}
	}
	else if (gHumanoid && mCameraAttachedToActor == gHumanoid->getHumanoidBody())
	{
		switch (ie.m_Id)
		{
		case HUMANOID_FORWARD:
			setFlag(gKeyFlags, Movement::eHUMANOID_FWD, val);
			break;
		case HUMANOID_BACKWARD:
			setFlag(gKeyFlags, Movement::eHUMANOID_BCKWD, val);
			break;
		case HUMANOID_LEFT:
			setFlag(gKeyFlags, Movement::eHUMANOID_ROTATE_LEFT, val);
			break;
		case HUMANOID_RIGHT:
			setFlag(gKeyFlags, Movement::eHUMANOID_ROTATE_RIGHT, val);
			break;
		}
	}

	if (val)
	{
		switch (ie.m_Id)
		{
		case CAMERA_SWITCH:
			{
				// cycle camera
				if(mHumanoidActor && mCameraAttachedToActor == NULL)
					mCameraAttachedToActor = mHumanoidActor;
				else if(gHumanoid && mCameraAttachedToActor != gHumanoid->getHumanoidBody())
					mCameraAttachedToActor = gHumanoid->getHumanoidBody();
				else
					mCameraAttachedToActor = NULL;

				mHumanoidCameraController->init(getCamera().getPos(), getCamera().getRot());
				mHumanoidCameraController->setFollowingMode(mCameraAttachedToActor != NULL);
			}
			break;
		case SCENE_RESET:
			{
				gResetScene = true;
			}
			break;
		}
	}

	PhysXSample::onDigitalInputEvent(ie,val);
}

void SampleHumanoid::onAnalogInputEvent(const SampleFramework::InputEvent& ie, float val)
{
	if (mHumanoidActor && mCameraAttachedToActor == mHumanoidActor)
	{
		switch (ie.m_Id)
		{
		case HUMANOID_FORWARD_BACKWARD:
			{
				setFlag(gKeyFlags, Movement::eSUBMAINE_FWD, val > 0.3f);
				setFlag(gKeyFlags, Movement::eSUBMAINE_BCKWD, val < -0.3f);
			}
			break;
		}
	}
	if(gHumanoid && mCameraAttachedToActor == gHumanoid->getHumanoidBody())
	{


	}
}

///////////////////////////////////////////////////////////////////////////////

void SampleHumanoid::handleInput()
{
	if(gHumanoid && mCameraAttachedToActor == gHumanoid->getHumanoidBody())
	{
		PxReal accFactor = 3.0f;
		PxReal forward = 0, rot = 0;
		if(gKeyFlags & Movement::eHUMANOID_FWD)
			forward = 1.0f;
		if(gKeyFlags & Movement::eHUMANOID_BCKWD)
			forward = -1.0f;
		if(gKeyFlags & Movement::eHUMANOID_ROTATE_LEFT)
			rot = 1.0f;
		if(gKeyFlags & Movement::eHUMANOID_ROTATE_RIGHT)
			rot = -1.0f;
		PxReal left = rot + forward; 
		PxReal right = -rot + forward;
		gHumanoid->setAcceleration(left*accFactor, right*accFactor);
	}
	else if(mHumanoidActor && mCameraAttachedToActor == mHumanoidActor)
	{
		if(gKeyFlags & Movement::eSUBMAINE_FWD)
			gForce.x -= gLinPower;
		if(gKeyFlags & Movement::eSUBMAINE_BCKWD)
			gForce.x += gLinPower;
		if(gKeyFlags & Movement::eSUBMAINE_UP)
			gForce.y += gLinPower;
		if(gKeyFlags & Movement::eSUBMAINE_DOWN)
			gForce.y -= gLinPower;

		if(gKeyFlags & (Movement::eSUBMAINE_FWD|Movement::eSUBMAINE_BCKWD))
		{
			PxSceneReadLock scopedLock(*mScene);

			static const PxReal camEpsilon = 0.001f;
			PxTransform subPose = mHumanoidActor->getGlobalPose();
			PxVec3 cameraDir = getCamera().getViewDir();
			PxVec3 cameraDirInSub = subPose.rotateInv(cameraDir).getNormalized();
			PxVec3 cameraUpInSub = subPose.rotateInv(PxVec3(0,1,0)).getNormalized();

			if(PxAbs(cameraDirInSub.z) > camEpsilon)
				gTorque.y += gAngPower*cameraDirInSub.z;
			if(PxAbs(cameraDirInSub.y) > camEpsilon)
				gTorque.z -= gAngPower*cameraDirInSub.y;
			if(PxAbs(cameraUpInSub.z) > camEpsilon)
				gTorque.x += gAngPower*cameraUpInSub.z;	
		}
	}
}

//////////////////////////////////////////////////////////////////////////

