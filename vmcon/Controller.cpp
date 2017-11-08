#include "Controller.h"
#include "MusculoSkeletalSystem.h"
#include "IKOptimization.h"

using namespace FEM;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Ipopt;

Controller::
Controller(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system)
	:mSoftWorld(soft_world),mRigidWorld(rigid_world),mMusculoSkeletalSystem(musculo_skeletal_system)
{
	int dof = mMusculoSkeletalSystem->GetSkeleton()->getNumDofs();
	double k = 500;

	mKp = Eigen::VectorXd::Constant(dof,k);
	mKv = Eigen::VectorXd::Constant(dof,2*sqrt(k));

	mTargetPositions = Eigen::VectorXd::Constant(dof,0.0);
	mTargetVelocities = Eigen::VectorXd::Constant(dof,0.0);

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
}
std::shared_ptr<Controller>
Controller::
Clone(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system)
{
	auto new_con = Create(soft_world,rigid_world,musculo_skeletal_system);

	new_con->mKp = mKp;
	new_con->mKv = mKv;

	new_con->mTargetPositions = mTargetPositions;
	new_con->mTargetVelocities = mTargetVelocities;
	return new_con;
}
std::shared_ptr<Controller>
Controller::
Create(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system)
{
	auto con = new Controller(soft_world,rigid_world,musculo_skeletal_system);

	return std::shared_ptr<Controller>(con);
}


Eigen::VectorXd
Controller::
ComputePDForces()
{
	auto& skel =mMusculoSkeletalSystem->GetSkeleton();

	Eigen::VectorXd pos_m = mTargetPositions;
	Eigen::VectorXd vel_m = mTargetVelocities;

	Eigen::VectorXd pos = skel->getPositions();
	Eigen::VectorXd vel = skel->getVelocities();

	for(int i = 0;i<pos.rows();i++)
		pos[i] = dart::math::wrapToPi(pos[i]);
	skel->setPositions(pos);
	Eigen::VectorXd pos_diff(pos.rows());

	pos_diff = skel->getPositionDifferences(pos_m,pos);


	Eigen::VectorXd qdd_desired = 
				pos_diff.cwiseProduct(mKp)+
				(vel_m - vel).cwiseProduct(mKv);
	return qdd_desired;
}

void
Controller::
AddIKTarget(AnchorPoint ap,const Eigen::Vector3d& target)
{
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
	ik->AddTargetPositions(ap,target);	
}
void
Controller::
SolveIK()
{
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));

	mIKSolver->OptimizeTNLP(mIKOptimization);

	mTargetPositions = ik->GetSolution();
}