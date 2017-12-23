#include "Controller.h"
#include "MusculoSkeletalSystem.h"
#include "IKOptimization.h"
#include "MuscleOptimization.h"
#include "FSM/Machine.h"
#include <fstream>

using namespace FEM;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Ipopt;

Controller::
Controller(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,const std::vector<std::shared_ptr<Ball>>& balls)
	:mSoftWorld(soft_world),mRigidWorld(rigid_world),mMusculoSkeletalSystem(musculo_skeletal_system),mBalls(balls)
{
	int dof = mMusculoSkeletalSystem->GetSkeleton()->getNumDofs();
	double k = 800;

	mKp = Eigen::VectorXd::Constant(dof,k);
	mKv = Eigen::VectorXd::Constant(dof,2*sqrt(k));
	for(int i =0;i<6;i++)
	{
		mKp[dof-1-i] = 2.0*mKp[dof-1-i];	
		mKv[dof-1-i] = sqrt(2.0)*mKv[dof-1-i];
	}
	
	
	mTargetPositions = Eigen::VectorXd::Constant(dof,0.0);
	mTargetVelocities = Eigen::VectorXd::Constant(dof,0.0);
	mPDForces = Eigen::VectorXd::Constant(dof,0.0);
	mIKOptimization = new IKOptimization(mMusculoSkeletalSystem->GetSkeleton());

	mIKSolver = new IpoptApplication();
	mIKSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	mIKSolver->Options()->SetStringValue("jac_c_constant", "yes");
	mIKSolver->Options()->SetStringValue("hessian_constant", "yes");
	mIKSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	mIKSolver->Options()->SetIntegerValue("print_level", 2);
	mIKSolver->Options()->SetIntegerValue("max_iter", 1000);
	mIKSolver->Options()->SetNumericValue("tol", 1e-3);

	mIKSolver->Initialize();

	mMuscleOptimization = new MuscleOptimization(mSoftWorld,mRigidWorld,mMusculoSkeletalSystem);
	mMuscleOptimizationSolver = new IpoptApplication();
	
	mMuscleOptimizationSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	mMuscleOptimizationSolver->Options()->SetStringValue("jac_c_constant", "no");
	mMuscleOptimizationSolver->Options()->SetStringValue("hessian_constant", "yes");
	mMuscleOptimizationSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	mMuscleOptimizationSolver->Options()->SetIntegerValue("print_level", 2);
	mMuscleOptimizationSolver->Options()->SetIntegerValue("max_iter", 100);
	mMuscleOptimizationSolver->Options()->SetNumericValue("tol", 1e-4);
	// std::vector<int> V_list{3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3};
		// std::vector<int> V_list{
		// 3,3,3,0,0,0,3,3,3,0,0,0};
	// std::vector<int> V_list{
	// 	3,3,3,3,3,
	// 	3,3,3,4,4,
	// 	4,4,4,4,4,
	// 	4,4,4,5,5,
	// 	5,5,5,5,5,
	// 	5,5,5,5,5,
	// 	5,3,3,3,3,
	// 	3,3,0,0,0,0,0,0,0};

	std::vector<int> V_list{5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
	// std::vector<int> V_list{1,1,1,1,1,1,1,1,1,1};
	// std::vector<int> V_list{5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1,5,3,1};
	mMuscleOptimizationSolver->Initialize();
		mMuscleOptimizationSolver->OptimizeTNLP(mMuscleOptimization);
	mFSM = std::make_shared<Machine>(mRigidWorld,mSoftWorld,mMusculoSkeletalSystem,mBalls,
		V_list,mBalls.size());

    // MakeMachine("../vmcon/export/juggling.xml",mFSM);
    // mFSM->Trigger("start");
}
std::shared_ptr<Controller>
Controller::
Clone(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,const std::vector<std::shared_ptr<Ball>>& balls)
{
	auto new_con = Create(soft_world,rigid_world,musculo_skeletal_system,balls);

	new_con->mKp = mKp;
	new_con->mKv = mKv;

	new_con->mTargetPositions = mTargetPositions;
	new_con->mTargetVelocities = mTargetVelocities;
	return new_con;
}
std::shared_ptr<Controller>
Controller::
Create(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,const std::vector<std::shared_ptr<Ball>>& balls)
{
	auto con = new Controller(soft_world,rigid_world,musculo_skeletal_system,balls);

	return std::shared_ptr<Controller>(con);
}
Eigen::VectorXd
Controller::
ComputeActivationLevels()
{
	auto& skel =mMusculoSkeletalSystem->GetSkeleton();
	Eigen::VectorXd qdd_desired = ComputePDForces();

	static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->Update(qdd_desired);

	mMuscleOptimizationSolver->ReOptimizeTNLP(mMuscleOptimization);	

	Eigen::VectorXd solution =  static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->GetSolution();
	Eigen::VectorXd qdd = solution.head(skel->getNumDofs());
	Eigen::VectorXd activation = solution.tail(mMusculoSkeletalSystem->GetNumMuscles());

	std::ofstream ofs1,ofs2,ofs3;
	ofs1.open("Desired_QDD.txt", std::ofstream::out | std::ofstream::app);
	ofs1<<qdd_desired.transpose()<<std::endl;
	ofs2.open("Result_QDD.txt",std::ofstream::out|std::ofstream::app);
	ofs2<<qdd.transpose()<<std::endl<<std::endl;
	ofs3.open("Activation.txt",std::ofstream::out|std::ofstream::app);
	ofs3<<activation.transpose()<<std::endl;
	return activation;
}

void
Controller::
SetRandomTargetPositions()
{
	dart::math::seedRand();
	for(int i =mMusculoSkeletalSystem->GetSkeleton()->getNumDofs()-3;i<mMusculoSkeletalSystem->GetSkeleton()->getNumDofs()-2;i++)
	{
		double lo = mMusculoSkeletalSystem->GetSkeleton()->getDof(i)->getPositionLowerLimit();
		double up = mMusculoSkeletalSystem->GetSkeleton()->getDof(i)->getPositionUpperLimit();

		mTargetPositions[i] = dart::math::random(lo,up);
	}
	// std::cout<<mTargetPositions.transpose()<<std::endl;
	// mTargetPositions.block(0,0,mMusculoSkeletalSystem->GetSkeleton()->getNumDofs()-3,1).setZero();


	// mMusculoSkeletalSystem->GetSkeleton()->setPositions(mTargetPositions);
	// Eigen::Quaterniond target_orientation;
	// target_orientation = Eigen::AngleAxisd(1.57,Eigen::Vector3d(-1.0,0,0));

	// Eigen::Quaterniond current_orientation(mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandL")->getTransform().rotation());
	// Eigen::Quaterniond diff = target_orientation*current_orientation.inverse();

	// std::cout<<Eigen::AngleAxisd(diff).angle()*Eigen::AngleAxisd(diff).axis().transpose()<<std::endl;

	mTargetVelocities.setZero();
}
Eigen::VectorXd
Controller::
ComputePDForces()
{
	auto& skel =mMusculoSkeletalSystem->GetSkeleton();
	mFSM->GetMotion(mTargetPositions,mTargetVelocities);

	Eigen::VectorXd pos_m = mTargetPositions;
	Eigen::VectorXd vel_m = mTargetVelocities;

	Eigen::VectorXd pos = skel->getPositions();
	Eigen::VectorXd vel = skel->getVelocities();

	Eigen::VectorXd pos_diff(pos.rows());

	pos_diff = skel->getPositionDifferences(pos_m,pos);
	for(int i = 0;i<pos_diff.rows();i++)
			pos_diff[i] = dart::math::wrapToPi(pos_diff[i]);

	Eigen::VectorXd qdd_desired = 
				pos_diff.cwiseProduct(mKp)+(vel_m - vel).cwiseProduct(mKv);
				
	mPDForces = qdd_desired;
	// std::cout<<pos_diff.transpose()<<std::endl;
	// std::cout<<(vel_m - vel).transpose()<<std::endl;
	// std::cout<<pos_diff.cwiseProduct(mKp).transpose()<<std::endl;
	// std::cout<<(vel_m - vel).cwiseProduct(mKv).transpose()<<std::endl;
	// std::cout<<std::endl;
	return qdd_desired;
}

void
Controller::
AddIKTarget(AnchorPoint ap,const Eigen::Vector3d& target)
{
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
	ik->AddTargetPositions(ap,target);	
}
Eigen::VectorXd
Controller::
SolveIK()
{
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));

	mIKSolver->OptimizeTNLP(mIKOptimization);

	return ik->GetSolution();
}

void
Controller::
Step()
{
#ifndef USE_JOINT_TORQUE
	mMusculoSkeletalSystem->SetActivationLevels(ComputeActivationLevels());
#else
	ComputePDForces();
#endif
}
