#ifndef __INTEGRATED_WORLD_H__
#define __INTEGRATED_WORLD_H__

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"

#include "fem/fem.h"

class MusculoSkeletalSystem;
class Ball;
class IntegratedWorld
{
public:
	IntegratedWorld(const std::string& path);

	const FEM::WorldPtr& GetSoftWorld(){return mSoftWorld;};	
	const dart::simulation::WorldPtr& GetRigidWorld() {return mRigidWorld;};
private:
	void Initialize(const std::string& path);
	FEM::WorldPtr mSoftWorld;
	dart::simulation::WorldPtr mRigidWorld;
	std::shared_ptr<MusculoSkeletalSystem> mMusculoSkeletalSystem;
	std::vector<std::shared_ptr<Ball>>					mBalls;
};

// void MakeSkeleton(dart::simulation::WorldPtr& world);
// void MakeBalls(dart::simulation::WorldPtr& world,int num);
#endif