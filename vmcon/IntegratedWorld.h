#ifndef __INTEGRATED_WORLD_H__
#define __INTEGRATED_WORLD_H__
#include "MusculoSkeletalSystem.h"
class IntegratedWorld
{
public:
	void Initialize();
	bool TimeStepping();

	IntegratedWorld(const IntegratedWorld& other) = delete;
	IntegratedWorld& operator=(const IntegratedWorld& other) = delete;
	std::shared_ptr<IntegratedWorld> Clone();
	static std::shared_ptr<IntegratedWorld> Create();

	const FEM::WorldPtr& GetSoftWorld(){return mSoftWorld;};	
	const dart::simulation::WorldPtr& GetRigidWorld() {return mRigidWorld;};
	const std::shared_ptr<MusculoSkeletalSystem>& GetMusculoSkeletalSystem(){return mMusculoSkeletalSystem;};
private:
	IntegratedWorld();

	FEM::WorldPtr mSoftWorld;
	dart::simulation::WorldPtr mRigidWorld;
	std::shared_ptr<MusculoSkeletalSystem> mMusculoSkeletalSystem;

};


#endif