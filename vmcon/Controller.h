#ifndef __CONTROLLER__H__
#define __CONTROLLER__H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "fem/fem.h"

struct Muscle;
class MusculoSkeletalSystem;

class Controller
{
public:
	Controller(const Controller& other) = delete;
	Controller& operator=(const Controller& other) = delete;
	std::shared_ptr<Controller> Clone(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system);
	static std::shared_ptr<Controller> Create(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system);

	Eigen::VectorXd ComputePDForces();
public:
	Controller(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system);

	Eigen::VectorXd 							mKp,mKv;
	Eigen::VectorXd								mTargetPositions;
	Eigen::VectorXd								mTargetVelocities;

	FEM::WorldPtr 								mSoftWorld;
	dart::simulation::WorldPtr 					mRigidWorld;
	std::shared_ptr<MusculoSkeletalSystem> 		mMusculoSkeletalSystem;
};





#endif