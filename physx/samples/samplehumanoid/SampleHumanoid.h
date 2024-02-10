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


#ifndef SAMPLE_HUMANOID_H
#define SAMPLE_HUMANOID_H

#include "PhysXSample.h"
#include "PxSimulationEventCallback.h"

namespace physx
{
	class PxJoint;
}

class Humanoid;
class HumanoidCameraController;

struct ClassType
{
	enum Type
	{
		eHUMANOID,
	};

	ClassType(const PxU32 type): mType(type)	{}
	PxU32 getType() const						{ return mType; }

	const PxU32 mType;
private:
	ClassType& operator=(const ClassType&);
};

struct FilterGroup
{
	enum Enum
	{
		eHUMANOID		= (1 << 0),
	};
};

class SampleHumanoid : public PhysXSample, public PxSimulationEventCallback
{
	friend class Humanoid;
	public:
											SampleHumanoid(PhysXSampleApplication& app);
	virtual									~SampleHumanoid();

	///////////////////////////////////////////////////////////////////////////////

	// Implements PxSimulationEventCallback
	virtual void							onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs);
	virtual void							onTrigger(PxTriggerPair* pairs, PxU32 count);
	virtual void							onConstraintBreak(PxConstraintInfo*, PxU32) {}
	virtual void							onWake(PxActor** , PxU32 ) {}
	virtual void							onSleep(PxActor** , PxU32 ){}
	virtual void							onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) {}

	///////////////////////////////////////////////////////////////////////////////

	// Implements SampleApplication
	virtual	void							onInit();
    virtual	void						    onInit(bool restart) { onInit(); }
	virtual	void							onShutdown();
	virtual	void							onTickPreRender(float dtime);
	virtual void							onDigitalInputEvent(const SampleFramework::InputEvent& , bool val);
	virtual void							onAnalogInputEvent(const SampleFramework::InputEvent& , float val);
	virtual void							onPointerInputEvent(const SampleFramework::InputEvent& ie, physx::PxU32 x, physx::PxU32 y, physx::PxReal dx, physx::PxReal dy, bool val);

	///////////////////////////////////////////////////////////////////////////////

	// Implements PhysXSampleApplication
	virtual void							helpRender(PxU32 x, PxU32 y, PxU8 textAlpha);
	virtual	void							descriptionRender(PxU32 x, PxU32 y, PxU8 textAlpha);
	virtual	void							customizeSample(SampleSetup&);
	virtual	void							customizeSceneDesc(PxSceneDesc&);
	virtual void							customizeRender();
	
	// called at end of a simulation substep
	virtual	void							onSubstep(float dtime);
	// called before the simulation begins, useful for setting up 
	// tasks that need to finish before the completionTask can be called
	virtual void							onSubstepSetup(float dtime, PxBaseTask* completionTask);	
	// called once the simulation has begun running
	virtual	void							onSubstepStart(float dtime);

	virtual void							collectInputEvents(std::vector<const SampleFramework::InputEvent*>& inputEvents);

	///////////////////////////////////////////////////////////////////////////////

	void                createMaterials();
	PxRigidDynamic* 	createHumanoid(const PxVec3& inPosition, const PxReal yRot);

	void				handleInput();
	PxRigidActor*		loadTerrain(const char* name, const PxReal heightScale, const PxReal rowScale, const PxReal columnScale);

	void				createDynamicActors();
	void				resetScene();

	std::vector<void*>&	getHumanoidsMemoryDeleteList() { return mHumanoidsMemoryDeleteList; }

	private:
			std::vector<PxJoint*>			mJoints;
			std::vector<Humanoid*>				mHumanoids;
			std::vector<void*>				mHumanoidsMemoryDeleteList;
			PxRigidDynamic*					mHumanoidActor;
			PxRigidDynamic*					mCameraAttachedToActor;

			HumanoidCameraController*		mHumanoidCameraController;
};

#endif
