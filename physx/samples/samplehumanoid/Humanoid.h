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

#ifndef HUMANOID_H
#define HUMANOID_H

#include "foundation/Px.h"
#include "foundation/PxSimpleTypes.h"
#include "common/PxPhysXCommonConfig.h"
#include "task/PxTask.h"
#include "SampleAllocator.h"
#include <vector>
#include <iostream>
#include <sstream>
#include <string>

#include "tinyxml2.h"
#include "MujocoXmlParser.h"

using namespace tinyxml2;
using namespace std;

class SampleHumanoid;

namespace physx
{
	class PxRigidDynamic;
	class PxRevoluteJoint;
}

 //  Edge Labels for Theo Jansen's Mechanism
 //		       _.
 //		     ,' |       m
 //		   ,'   |2b   .....
 //		 ,' 2a  |    e    |n
 //		+-------+---------+
 //		|       |
 //		|       |2c
 //		|       |
 //		+-------+
 //		 `.     |
 //		   `.   |2d
 //		     `. |
 //		       `|

struct HumanoidState
{
	enum Enum
	{
		eWAITING,
		eMOVE_FWD,
		eMOVE_BKWD,
		eROTATE_LEFT,
		eROTATE_RIGHT,
		ePANIC,
		eNUM_STATES,
	};
};


class Humanoid: public ClassType, public physx::PxLightCpuTask, public SampleAllocateable
{
public:
	Humanoid(SampleHumanoid& sample, const PxVec3& crabPos, RenderMaterial* material);
	Humanoid(SampleHumanoid& sample, const char* filename,  RenderMaterial* material);
	~Humanoid();

	void	update(PxReal dt);
	void	setAcceleration(PxReal leftAcc, PxReal rightAcc);
	void	flushAccelerationBuffer();

	PX_INLINE const PxRigidDynamic*		getHumanoidBody() const		{ return mHumanoidBody; }
	PX_INLINE PxRigidDynamic*			getHumanoidBody()			{ return mHumanoidBody; }
	PX_INLINE bool						needsRespawn()			{ return mRespawnMe; }
	void	save(const char* filename);

	// Implements LightCpuTask
	virtual  const char*				getName() const			{ return "Humanoid AI Task"; }
	virtual  void						run();

private:
	void	initMembers();
	void	create(const PxVec3& _crabPos);
	void	load(const char* filename);

	PxVec3	getPlaceOnFloor(PxVec3 pos);
	void	createLeg(PxRigidDynamic* mainBody, PxVec3 localPos, PxReal mass, PxReal scale, PxRigidDynamic* motor, PxVec3 motorAttachmentPos);
	int		loadMujoco();
	void	ProcessMujoco(BodyParam* _bodyParam, PxRigidDynamic* _bodyParent);
	void	createTorso(PxRigidDynamic* mainBody, PxVec3 localPos, PxRigidDynamic* motor, PxVec3 motorAttachmentPos);

	void	updateState();
	void	initState(HumanoidState::Enum state);

public:
	std::vector<PxRigidDynamic*>		mActors;

private:
	SampleHumanoid*					mSampleHumanoid;

	PxRigidDynamic*						mHumanoidBody;
	PxRevoluteJoint*					mMotorJoint[2];
	
	std::vector<PxJoint*>				mJoints;


	RenderMaterial*					mMaterial;
	PxReal							mAcceleration[2];
	PxReal							mLegHeight;
	bool							mRespawnMe;

	HumanoidState::Enum					mHumanoidState;
	PxReal							mStateTime;
	PxReal							mDistances[10];
	PxReal							mAccumTime;
	PxReal							mElapsedTime;
	PxVec3							mHumanoidPos;
	PxReal							mAccelerationBuffer[2];
	
	volatile PxU32					mRunning;
	void*							mMemory;
};

#endif
