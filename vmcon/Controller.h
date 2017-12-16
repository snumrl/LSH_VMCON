#ifndef __CONTROLLER__H__
#define __CONTROLLER__H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"

#include "fem/fem.h"

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>	
#include "Ball.h"
struct Muscle;
class Machine;
class MusculoSkeletalSystem;
class MuscleOptimization;
typedef std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d> AnchorPoint;

class Controller
{
public:
	Controller(const Controller& other) = delete;
	Controller& operator=(const Controller& other) = delete;
	std::shared_ptr<Controller> Clone(
		const FEM::WorldPtr& soft_world,
		const dart::simulation::WorldPtr& rigid_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls);
	static std::shared_ptr<Controller> Create(
		const FEM::WorldPtr& soft_world,
		const dart::simulation::WorldPtr& rigid_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls);

	Eigen::VectorXd ComputePDForces();
	Eigen::VectorXd ComputeActivationLevels();

	// void AddIKTarget(AnchorPoint ap,const Eigen::Vector3d& target);
	// Eigen::VectorXd SolveIK();

	void Step();
	const Eigen::VectorXd& GetTargetPositions() {return mTargetPositions;}
	void SetTargetPositions(const Eigen::VectorXd& x) { mTargetPositions=x;}

	Eigen::VectorXd mPDForces;
public:
	Controller(
		const FEM::WorldPtr& soft_world,
		const dart::simulation::WorldPtr& rigid_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls);

	Eigen::VectorXd 									mKp,mKv;
	Eigen::VectorXd										mTargetPositions;
	Eigen::VectorXd										mTargetVelocities;

	FEM::WorldPtr 										mSoftWorld;
	dart::simulation::WorldPtr 							mRigidWorld;
	std::shared_ptr<MusculoSkeletalSystem> 				mMusculoSkeletalSystem;
	std::vector<std::shared_ptr<Ball>>					mBalls;

	// Ipopt::SmartPtr<Ipopt::TNLP>			 			mIKOptimization;
	// Ipopt::SmartPtr<Ipopt::IpoptApplication> 			mIKSolver;

	Ipopt::SmartPtr<Ipopt::TNLP> 			 			mMuscleOptimization;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> 			mMuscleOptimizationSolver;

	std::shared_ptr<Machine> 							mFSM;
};


#endif