#include "Controller.h"
#include "MusculoSkeletalSystem.h"
#include "IKOptimization.h"
#include "MuscleOptimization.h"
#include "FSM.h"
using namespace FEM;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Ipopt;

Controller::
Controller(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,const std::vector<std::shared_ptr<Ball>>& balls)
	:mSoftWorld(soft_world),mRigidWorld(rigid_world),mMusculoSkeletalSystem(musculo_skeletal_system),mBalls(balls)
{
	int dof = mMusculoSkeletalSystem->GetSkeleton()->getNumDofs();
	double k = 500;

	mKp = Eigen::VectorXd::Constant(dof,k);
	mKv = Eigen::VectorXd::Constant(dof,2*sqrt(k));
	mKp[dof-2] = 2.0*mKp[dof-2];
	mKp[dof-1] = 2.0*mKp[dof-1];
	mKv[dof-2] = sqrt(2.0)*mKv[dof-1];
	mKv[dof-1] = sqrt(2.0)*mKv[dof-1];
	mTargetPositions = Eigen::VectorXd::Constant(dof,0.0);
	mTargetVelocities = Eigen::VectorXd::Constant(dof,0.0);
	mPDForces = Eigen::VectorXd::Constant(dof,0.0);
	// mIKOptimization = new IKOptimization(mMusculoSkeletalSystem->GetSkeleton());

	// mIKSolver = new IpoptApplication();
	// mIKSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	// mIKSolver->Options()->SetStringValue("jac_c_constant", "yes");
	// mIKSolver->Options()->SetStringValue("hessian_constant", "yes");
	// mIKSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	// mIKSolver->Options()->SetIntegerValue("print_level", 2);
	// mIKSolver->Options()->SetIntegerValue("max_iter", 1000);
	// mIKSolver->Options()->SetNumericValue("tol", 1e-3);

	// mIKSolver->Initialize();

	mMuscleOptimization = new MuscleOptimization(mSoftWorld,mRigidWorld,mMusculoSkeletalSystem);
	mMuscleOptimizationSolver = new IpoptApplication();
	
	mMuscleOptimizationSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	mMuscleOptimizationSolver->Options()->SetStringValue("jac_c_constant", "no");
	mMuscleOptimizationSolver->Options()->SetStringValue("hessian_constant", "yes");
	mMuscleOptimizationSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	mMuscleOptimizationSolver->Options()->SetIntegerValue("print_level", 2);
	mMuscleOptimizationSolver->Options()->SetIntegerValue("max_iter", 100);
	mMuscleOptimizationSolver->Options()->SetNumericValue("tol", 1e-4);
	mFSM = std::make_shared<Machine>(mRigidWorld,mSoftWorld,mMusculoSkeletalSystem,mBalls,std::shared_ptr<Controller>(this),mSoftWorld->GetTimeStep());
    MakeMachine("../vmcon/export/juggling.xml",mFSM);
    mFSM->Trigger("start");
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

	if(mSoftWorld->GetTime()==0.0)
	{
		mMuscleOptimizationSolver->Initialize();
		mMuscleOptimizationSolver->OptimizeTNLP(mMuscleOptimization);
	}
	else
		mMuscleOptimizationSolver->ReOptimizeTNLP(mMuscleOptimization);	

	Eigen::VectorXd solution =  static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->GetSolution();
	Eigen::VectorXd qdd = solution.head(skel->getNumDofs());
	Eigen::VectorXd activation = solution.tail(mMusculoSkeletalSystem->GetNumMuscles());

//	std::cout<<"desired qdd :"<<qdd_desired.transpose()<<std::endl;
//	std::cout<<"result  qdd :"<<qdd.transpose()<<std::endl<<std::endl;
	// std::cout<<"activation  :"<<activation.transpose()<<std::endl;
	return activation;
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

// void
// Controller::
// AddIKTarget(AnchorPoint ap,const Eigen::Vector3d& target)
// {
// 	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
// 	ik->AddTargetPositions(ap,target);	
// }
// Eigen::VectorXd
// Controller::
// SolveIK()
// {
// 	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));

// 	mIKSolver->OptimizeTNLP(mIKOptimization);

// 	return ik->GetSolution();
// }

void
Controller::
Step()
{
	
	// pd_forces = mMusculoSkeletalSystem->GetSkeleton()->getMassMatrix()*pd_forces + mMusculoSkeletalSystem->GetSkeleton()->getCoriolisAndGravityForces();
	// mMusculoSkeletalSystem->GetSkeleton()->setForces(pd_forces);
	
<<<<<<< HEAD
	mPDForces = ComputePDForces();

=======
	
	ComputePDForces();
>>>>>>> ed9b5dfd6354f6f33a6820d7ad02e239d91e2ca6
	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getPositions().transpose()<<std::endl;
	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getVelocities().transpose()<<std::endl;
	// std::cout<<mTargetPositions.transpose()<<std::endl;
	// std::cout<<mTargetVelocities.transpose()<<std::endl;
	// std::cout<<mPDForces.transpose()<<std::endl;
	// std::cout<<std::endl;
	// mMusculoSkeletalSystem->SetActivationLevels(mMusculoSkeletalSystem->GetActivationLevels().setZero());
<<<<<<< HEAD
	// mMusculoSkeletalSystem->SetActivationLevels(ComputeActivationLevels());
}
=======
	//mMusculoSkeletalSystem->SetActivationLevels(ComputeActivationLevels());
}
>>>>>>> ed9b5dfd6354f6f33a6820d7ad02e239d91e2ca6
