#include "Controller.h"
#include "Ball.h"
#include "MusculoSkeletalSystem.h"
#include "IKOptimization.h"
#include "MuscleOptimization.h"
#include "BezierCurve.h"

using namespace FEM;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Ipopt;

Controller::
Controller(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,const std::vector<std::shared_ptr<Ball>>& balls)
	:mSoftWorld(soft_world),mRigidWorld(rigid_world),mMusculoSkeletalSystem(musculo_skeletal_system),mBalls(balls)
{
	int dof = mMusculoSkeletalSystem->GetSkeleton()->getNumDofs();
	double k = 300;

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

	mMuscleOptimization = new MuscleOptimization(mSoftWorld,mRigidWorld,mMusculoSkeletalSystem);
	mMuscleOptimizationSolver = new IpoptApplication();
	
	mMuscleOptimizationSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	mMuscleOptimizationSolver->Options()->SetStringValue("jac_c_constant", "no");
	mMuscleOptimizationSolver->Options()->SetStringValue("hessian_constant", "yes");
	mMuscleOptimizationSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	mMuscleOptimizationSolver->Options()->SetIntegerValue("print_level", 2);
	mMuscleOptimizationSolver->Options()->SetIntegerValue("max_iter", 100);
	mMuscleOptimizationSolver->Options()->SetNumericValue("tol", 1e-4);

	std::vector<int> V = {1};
	mJuggling.GenerateStates(V);

	MakeJugglingFSM(mFSM[0]);
	MakeJugglingFSM(mFSM[1]);

	Eigen::VectorXd init_pos = mMusculoSkeletalSystem->GetSkeleton()->getPositions();
	Swing(0);
	// mMotion.push_back(std::make_pair(0.0,init_pos*1.0));
	// mMotion.push_back(std::make_pair(1.0,Eigen::VectorXd::Zero(init_pos.rows())));


}
std::shared_ptr<Controller>
Controller::
Clone(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,const std::vector<std::shared_ptr<Ball>>& balls)
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
Create(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,const std::vector<std::shared_ptr<Ball>>& balls)
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

	// std::cout<<"desired qdd :"<<qdd_desired.transpose()<<std::endl;
	// std::cout<<"result  qdd :"<<qdd.transpose()<<std::endl;
	// std::cout<<"activation  :"<<activation.transpose()<<std::endl;
	return activation;
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

	// for(int i = 0;i<pos.rows();i++)
	// 	pos[i] = dart::math::wrapToPi(pos[i]);
	// skel->setPositions(pos);
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
Eigen::VectorXd
Controller::
SolveIK()
{
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));

	mIKSolver->OptimizeTNLP(mIKOptimization);

	return ik->GetSolution();
}
bool
Controller::
CheckFSM()
{
	bool need_update = false;
	int ji = mJuggling.mCurrent;

	bool check_hand[2] = {false,false};

	for(int i = ji;i < mJuggling.mStateSequences.size();i++)
	{		
		int hand = mJuggling.mStateSequences[i].hand;
		check_hand[hand] = true;

		const auto& interest_ball = mBalls[mJuggling.mStateSequences[i].ball];

		if(interest_ball->IsAttached())
		{

		}





		if(check_hand[0]&&check_hand[1])
			break;
	}

	return need_update;
}
void
Controller::
UpdateTarget()
{
	double cur_time = mRigidWorld->getTime();
	double dt = 0.01;
	int looking,looking_next;
	for(looking =0;looking<mMotion.size();looking++)
	{
		if(cur_time<mMotion[looking].first)
			break;
	}
	looking--;

	for(looking_next =0;looking_next<mMotion.size();looking_next++)
	{
		if(cur_time+dt<mMotion[looking_next].first)
			break;
	}

	looking_next--;

	double k = cur_time - mMotion[looking].first;
	double dk = mMotion[looking+1].first - mMotion[looking].first;
	k /= dk;

	double k_next = cur_time+dt - mMotion[looking_next].first;
	double dk_next =mMotion[looking_next+1].first - mMotion[looking_next].first;
	k_next /= dk_next;

	mTargetPositions = mMotion[looking].second *(1-k) + mMotion[looking+1].second *(k);
	auto next_target_positions = mMotion[looking_next].second *(1-k_next) + mMotion[looking_next+1].second *(k_next);

	mTargetVelocities = (next_target_positions - mTargetPositions)/dt;
}
void
Controller::
Swing(int hand)
{
	auto& skel = mMusculoSkeletalSystem->GetSkeleton();
	auto ee = skel->getBodyNode(HAND_NAME[hand]);

	AnchorPoint ap = std::make_pair(ee,Eigen::Vector3d(0,0,0));

	BezierCurve bc(
		Eigen::Vector3d(0.6,1.0,0.4),
		Eigen::Vector3d(0.4,0.3,0.2),
		Eigen::Vector3d(0.3,-0.2,0),
		Eigen::Vector3d(0.1,0,0.2));

	for(double t = 0.0;t<=1.0;t+=0.1)
	{
		Eigen::Vector3d target = bc.GetPoint(t);
		AddIKTarget(ap,target);
		Eigen::VectorXd pos = SolveIK();
		mMotion.push_back(std::make_pair(t*0.5,pos));
	}
}
void
Controller::
Step()
{
	//Check Condition & Change FSM if needed.
	bool need_update = CheckFSM();
	UpdateTarget();
	mMusculoSkeletalSystem->SetActivationLevels(ComputeActivationLevels());
}