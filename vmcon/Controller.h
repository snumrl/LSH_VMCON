#ifndef __CONTROLLER__H__
#define __CONTROLLER__H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"

#include "fem/fem.h"

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>	

struct Muscle;
class MusculoSkeletalSystem;
typedef std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d> AnchorPoint;

class Controller
{
public:
	Controller(const Controller& other) = delete;
	Controller& operator=(const Controller& other) = delete;
	std::shared_ptr<Controller> Clone(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system);
	static std::shared_ptr<Controller> Create(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system);

	Eigen::VectorXd ComputePDForces();

	void AddIKTarget(AnchorPoint ap,const Eigen::Vector3d& target);
	void SolveIK();
public:
	Controller(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system);

	Eigen::VectorXd 									mKp,mKv;
	Eigen::VectorXd										mTargetPositions;
	Eigen::VectorXd										mTargetVelocities;

	FEM::WorldPtr 										mSoftWorld;
	dart::simulation::WorldPtr 							mRigidWorld;
	std::shared_ptr<MusculoSkeletalSystem> 				mMusculoSkeletalSystem;

	Ipopt::SmartPtr<Ipopt::TNLP>			 			mIKOptimization;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> 			mIKSolver;
};





#endif