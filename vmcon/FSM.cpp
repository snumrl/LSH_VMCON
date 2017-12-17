#include "FSM.h"
#include "DART_helper.h"
#include "Ball.h"
#include "Record.h"
#include "BezierCurve.h"
#include "IKOptimization.h"
#include "MusculoSkeletalSystem.h"
#include "iLQR/MusculoSkeletalLQR.h"
#include "Controller.h"
#include <tinyxml.h>
#include <iostream>
#include <fstream>
#include <string>
#include <tuple>
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Ipopt;

State::
State(const dart::simulation::WorldPtr& rigid_world,
				const FEM::WorldPtr& soft_world,
				const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
				const std::shared_ptr<Controller>& controller,
				dart::dynamics::BodyNode* bn,
				const std::vector<std::shared_ptr<Ball>>& balls,
				const Ipopt::SmartPtr<Ipopt::TNLP>& ik_optimization,
				const Ipopt::SmartPtr<Ipopt::IpoptApplication>& ik_solver)
	:mRigidWorld(rigid_world),
	mSoftWorld(soft_world),
	mMusculoSkeletalSystem(musculo_skeletal_system),
	mController(controller),
	mAnchorPoint(std::make_pair(bn,Eigen::Vector3d(0,0,0))),
	mBalls(balls),
	mBallIndex(-1),mV(0),
	mIKOptimization(ik_optimization),
	mIKSolver(ik_solver),
	mTimeElapsed(0.0)
{
	auto T_hb = bn->getTransform().inverse()*mBalls[0]->GetSkeleton()->getBodyNode(0)->getTransform();
	mAnchorPoint.second = T_hb.translation();
}
void
State::
AddEvent(const std::string& name,State* next_state)
{
	mEvents.insert(std::make_pair(name,next_state));	
}

State*
State::
GetNextState(const std::string& event_name)
{
	if(mEvents.find(event_name)==mEvents.end())
	{
		// std::cout<<"NO event name : "<<event_name<<std::endl;
		return nullptr;
	}
	return mEvents.at(event_name);
}
std::map<std::string,State*>&
State::
GetEvents()
{
	return mEvents;
}
void
State::
TimeStepping(double time_step)
{
	mTimeElapsed += time_step;
}



IKState::
IKState(const dart::simulation::WorldPtr& rigid_world,
				const FEM::WorldPtr& soft_world,
				const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
				const std::shared_ptr<Controller>& controller,
				dart::dynamics::BodyNode* bn,
				const std::vector<std::shared_ptr<Ball>>& balls,
				const Ipopt::SmartPtr<Ipopt::TNLP>& ik_optimization,
				const Ipopt::SmartPtr<Ipopt::IpoptApplication>& ik_solver)
	:State(rigid_world,soft_world,musculo_skeletal_system,controller,bn,balls,ik_optimization,ik_solver)
{

}
void
IKState::
Solve()
{
	// std::cout<<"SOLVE IK : "<<mBallIndex<<std::endl;
	bool need_ik_update= true;
	

	Eigen::Vector3d target;
	mBalls[mBallIndex]->ComputeFallingPosition(mBalls[mBallIndex]->releasedPoint[1],target);
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
	
	const auto& ik_targets = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization))->GetTargets();
	for(auto& t : ik_targets)
	{
		if(!t.first.first->getName().compare(mAnchorPoint.first->getName()))
		{
			if((t.second-target).norm()<5E-6)
				need_ik_update = false;
		}
	}
	if(!mBalls[mBallIndex]->isReleased)
		need_ik_update = false;
	if(need_ik_update)
	{
		// std::cout<<target.transpose()<<std::endl;
		if(target.norm()>1E-6){
			ik->AddTargetPositions(mAnchorPoint,target);
			mIKSolver->ReOptimizeTNLP(mIKOptimization);	
		}
		
	}
}
void 
IKState::
Initialize(int ball_index,int V)
{
	mBallIndex = ball_index;
	mV = V;
	Solve();
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
	mTimeElapsed=0.0;
}
std::string 
IKState::
GetMotion(Eigen::VectorXd& p,Eigen::VectorXd& v)
{
	std::string event("no_event");
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));

	Solve();
	Eigen::VectorXd sol = ik->GetSolution();
	p = sol;
	v.resize(p.rows());
	v.setZero();

	Eigen::Vector3d body_COM = mAnchorPoint.first->getCOM();
	Eigen::Vector3d ball_COM = mBalls[mBallIndex]->GetPosition();
	if((body_COM-ball_COM).norm()<5E-2){
		// std::cout<<"Attach "<<mBallIndex<<std::endl;
		mBalls[mBallIndex]->Attach(mRigidWorld,mAnchorPoint.first);
		// event = "catch";
	}

	return event;
}

Eigen::Vector3d
IKState::
GetTarget()
{
	const auto& ik_targets = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization))->GetTargets();
	for(auto& t : ik_targets)
		if(!t.first.first->getName().compare(mAnchorPoint.first->getName()))
			return t.second;
	return Eigen::Vector3d(0,0,0);
}

BezierCurveState::
BezierCurveState(const dart::simulation::WorldPtr& rigid_world,
				const FEM::WorldPtr& soft_world,
				const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
				const std::shared_ptr<Controller>& controller,
				dart::dynamics::BodyNode* bn,
				const std::vector<std::shared_ptr<Ball>>& balls,
				const Ipopt::SmartPtr<Ipopt::TNLP>& ik_optimization,
				const Ipopt::SmartPtr<Ipopt::IpoptApplication>& ik_solver,
			double D,double T,
			int num_curve_sample)
	:State(rigid_world,soft_world,musculo_skeletal_system,controller,bn,balls,ik_optimization,ik_solver),mCurve(BezierCurve())
	,mD(D),mT(T),mNumCurveSample(num_curve_sample)
{
	InitializeLQR();
}
extern int v_target_from_argv;
void 
BezierCurveState::
Initialize(int ball_index,int V)
{
	mCount = 0;
	mTimeElapsed = 0;
	mBallIndex = ball_index;
	mV = V;
	double t = ((double)mV-2*mD+0.1)*mT;

	Eigen::Vector3d p0,p1,p2,v2;
	p0.setZero();
	p1.setZero();
	p2.setZero();
	v2.setZero();

	
	bool isleft = false;
	if(mAnchorPoint.first->getName().find("L")!=std::string::npos)
		isleft = true;

	p0 = mBalls[mBallIndex]->GetPosition();
	p2 = p0;
	p2[2] += 0.05;
	// if(isleft){
	// 	p0[0] +=0.1;
	// 	p2[0] = p0[0]-0.15;
	// }
	// else{
	// 	p0[0] -=0.1;
	// 	p2[0] = p0[0]+0.15;
	// }
	// p0[1] +=0.2;
	// p2[1] = 0.1;
	// p2[2] = p0[2];

	

	v2[0] = -p0[0]/t;
	// v2[0] = 0;
	v2[1] = 0.5*9.81*t;

	v2.setZero();
	// v2[1] = 4;
	v2[1] = v_target_from_argv;
	v2[2] = -0.1;
	
	// v2 = v_target_from_argv;

	
	// v2 = Eigen::Vector3d(0,8,0);
	// p0[2] = 0.3;
	p1 = p2 - 0.5*mD*mT*v2;

	// std::cout<<"p0 : "<<p0.transpose()<<std::endl;
	// std::cout<<"p1 : "<<p1.transpose()<<std::endl;
	// std::cout<<"p2 : "<<p2.transpose()<<std::endl;
	// std::cout<<"v2 : "<<v2.transpose()<<std::endl;

	mCurve.Initialize(p0,p1,p2,mD*mT);

	
	GenerateMotions(mCurve,mMotions);
	Eigen::VectorXd p,v;
	std::vector<std::pair<Eigen::VectorXd,double>> fine_motions;
	mTimeElapsed=0.0;
	while(true)
	{
		int k =0,k1 =0;
		for(int i =0;i<mMotions.size();i++)
		{
			if(mMotions[i].second<mTimeElapsed)
				k=i;
		}
		if(k ==mMotions.size()-1)
			break;
		k1 = k+1;
		double t = mTimeElapsed-mMotions[k].second;
		double dt = mMotions[k1].second-mMotions[k].second;

		p = (1.0-t/dt)*(mMotions[k].first) + (t/dt)*(mMotions[k1].first);
		double t1 = t+0.001;
		auto pdp =(1.0-t1/dt)*(mMotions[k].first) + (t1/dt)*(mMotions[k1].first);

		v = (pdp-p)/0.001;
		fine_motions.push_back(std::make_pair(p,mTimeElapsed));
		mTimeElapsed+=mSoftWorld->GetTimeStep();
	}


	// while(GetMotion(p,v).compare("end"))
	// {
	// 	fine_motions.push_back(std::make_pair(p,mTimeElapsed));
	// 	mTimeElapsed += mSoftWorld->GetTimeStep();
	// }
	mBalls[mBallIndex]->Attach(mRigidWorld,mAnchorPoint.first);
	mCount=0;
	mTimeElapsed = 0.0;

	mMotions = fine_motions;
	// std::cout<<"FINE MOTION SIZE : "<<mMotions.size()<<std::endl;

	SynchronizeLQR();
	OptimizeLQR(p2,v2);

	// std::cout<<mMotions.size()<<std::endl;
	// std::cout<<mU.size()<<std::endl;
	// std::cout<<mU.size()<<std::endl;
	// for(int i =0;i<mMotions.size()-2;i++)
	// {
		// std::cout<<mMotions[i].first.rows()<<std::endl;
		// std::cout<<mU[i].transpose()<<std::endl;
		// mMotions[i].first = mMotions[i].first + mU[i];
	// }
}
void
BezierCurveState::
SynchronizeLQR()
{
	mLQRSoftWorld->SetPositions(mSoftWorld->GetPositions());
	// std::vector<bool> is_attacheds;
	// for(int i =0;i<mBalls.size();i++)
	// {
	// 	is_attacheds.push_back(mBalls[i]->IsAttached());
	// 	mLQRBalls[i]->Release(mLQRRigidWorld);
	// }
	for(int i =0;i<mLQRRigidWorld->getNumSkeletons();i++){
		mLQRRigidWorld->getSkeleton(i)->setPositions(mRigidWorld->getSkeleton(i)->getPositions());
		mLQRRigidWorld->getSkeleton(i)->setVelocities(mRigidWorld->getSkeleton(i)->getVelocities());
	}

	for(int i =0;i<mBalls.size();i++)
	{
		if(!mBalls[i]->IsReleased())
		{
			mLQRBalls[i]->Attach(mLQRRigidWorld,mLQRMusculoSkeletalSystem->GetSkeleton()->getBodyNode(mBalls[i]->GetConstraint()->getBodyNode2()->getName()));
		}
		else
		{
			mLQRBalls[i]->Release(mLQRRigidWorld);
		}
	}
}
void
BezierCurveState::
OptimizeLQR(const Eigen::Vector3d& p_des,const Eigen::Vector3d& v_des)
{
	int dofs =mLQRMusculoSkeletalSystem->GetSkeleton()->getNumDofs();
	Eigen::VectorXd x0(dofs*2+12*mBalls.size());
	x0.head(dofs) = mLQRMusculoSkeletalSystem->GetSkeleton()->getPositions();
	x0.block(dofs,0,dofs,1) = mLQRMusculoSkeletalSystem->GetSkeleton()->getVelocities();
	for(int i =0;i<mBalls.size();i++)
	{
		x0.block(2*dofs+12*i,0,6,1) = mLQRBalls[mBallIndex]->GetSkeleton()->getPositions();
		x0.block(2*dofs+12*i+6,0,6,1) = mLQRBalls[mBallIndex]->GetSkeleton()->getVelocities();
	}	
	std::vector<Eigen::VectorXd> ref,u0;
	ref.resize(mMotions.size()-1);
	u0.resize(mMotions.size()-1);


	for(int i=0;i<u0.size();i++){
		ref[i] = mMotions[i].first;
		u0[i] = ref[i];
		u0[i].setZero();
	}
	mLQR->Initialze(p_des,v_des,mBallIndex,ref,x0,u0);
	mU = mLQR->Solve();

	// std::cout<<std::endl;
	// for(int i=0;i<mU.size();i++)
	// {
		// std::cout<<mU[i].transpose()<<std::endl;
	// }
	// mU = u0;
}

void
BezierCurveState::
InitializeLQR()
{
	mLQRSoftWorld = FEM::World::Create(
		mSoftWorld->GetIntegrationMethod(),
		mSoftWorld->GetTimeStep(),
		mSoftWorld->GetMaxIteration());
	mLQRRigidWorld = std::make_shared<World>();
	mLQRRigidWorld->setGravity(Eigen::Vector3d(0,-9.81,0));

	mLQRMusculoSkeletalSystem = MusculoSkeletalSystem::Create();
	MakeSkeleton(mLQRMusculoSkeletalSystem);
	MakeMuscles("../vmcon/export/muscle_params.xml",mLQRMusculoSkeletalSystem);

	mLQRMusculoSkeletalSystem->Initialize(mLQRSoftWorld,mLQRRigidWorld);
	mLQRSoftWorld->Initialize();
	double ball_mass[10] = {0.13,0.5,1.0,3.0,5.0,10.0,20.0};

	for(int i =0;i<1;i++)
	{
		SkeletonPtr skel = Skeleton::create("ball_"+std::to_string(i));

		bool is_left_hand = i%2;
		if(is_left_hand)
		{
			auto* abn =mLQRMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandL");
			Eigen::Vector3d loc = abn->getTransform().translation();
			MakeBall(skel,abn->getCOM(),0.036,0.13);
			// MakeBall(skel,bp,0.036,ball_mass[v_target_from_argv]);

			mLQRBalls.push_back(std::make_shared<Ball>(nullptr,skel));
			mLQRRigidWorld->addSkeleton(skel);
			mLQRBalls.back()->Attach(mLQRRigidWorld,abn);
		}
		else
		{
			auto* abn =mLQRMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR");
			Eigen::Vector3d loc = abn->getTransform().translation();
			Eigen::Vector3d bp = abn->getCOM();
			// bp[1] -=0.02;
			// bp[1] +=0.06;
			// bp[0] += 1.0;
			bp[0] -= 0.03;
			bp[1] += 0.03;
			bp[2] += 0.02;
			// std::cout<<bp.transpose()<<std::endl;
			// MakeBall(skel,bp,0.036,ball_mass[v_target_from_argv]);
			MakeBall(skel,bp,0.036,0.13);
			mLQRBalls.push_back(std::make_shared<Ball>(nullptr,skel));
			mLQRRigidWorld->addSkeleton(skel);
			mLQRBalls.back()->Attach(mLQRRigidWorld,abn);	
		}
	}

	mLQR = std::make_shared<MusculoSkeletalLQR>(
			mLQRRigidWorld,
			mLQRSoftWorld,
			mLQRMusculoSkeletalSystem,mLQRBalls,10);
}


std::string 
BezierCurveState::
GetMotion(Eigen::VectorXd& p,Eigen::VectorXd& v)
{
	// int k =0,k1 =0;
	// for(int i =0;i<mMotions.size();i++)
	// {
	// 	if(mMotions[i].second<mTimeElapsed)
	// 		k=i;
	// }

	if(mCount >= mU.size()-1)
	{
		// std::cout<<"dasdf"<<std::endl;
		// std::cout<<k<<std::endl;
		// std::cout<<mCount<<std::endl;

		mBalls[mBallIndex]->Release(mRigidWorld);

		std::cout<<"Released Velocity : "<<mBalls[mBallIndex]->releasedVelocity.transpose()<<std::endl;

		p = mMotions.back().first+mU.back();
		v = mMotions[0].first;
		v.setZero();
		mCount++;
		if(mCount>=mU.size()+10)
			return std::string("end_same_hand");
		else
			return std::string("no_event");

	}

	// k1 = k+1;
	// double t = mTimeElapsed-mMotions[k].second;
	// double dt = mMotions[k1].second-mMotions[k].second;


	// // p = (1.0-t/dt)*(mMotions[k].first) + (t/dt)*(mMotions[k1].first);
	// p = mMotions[k].first;
	// if(k==0)
	// 	v = (mMotions[0].first-mMotions[0].first)/mSoftWorld->GetTimeStep();
	// else
	// 	v = (mMotions[k].first-mMotions[k-1].first)/mSoftWorld->GetTimeStep();
	// double t1 = t+0.001;
	// std::cout<<t<<std::endl;
	// std::cout<<t1<<std::endl;
	// auto pdp =(1.0-t1/dt)*(mMotions[k].first) + (t1/dt)*(mMotions[k1].first);

	// v = (pdp-p)/0.001;
	// std::cout<<mMotions.size()<<std::endl;
	// std::cout<<k<<std::endl;
	// std::cout<<mTimeElapsed<<std::endl;
	
	// std::cout<<p.transpose()<<std::endl;
	// std::cout<<v.transpose()<<std::endl;
	p = mMotions[mCount].first + mU[mCount];
	if(mCount == 0){
		v = (mMotions[mCount+1].first - mMotions[mCount].first)/mSoftWorld->GetTimeStep();
	}
	else{
		v = (p-(mMotions[mCount-1].first+mU[mCount-1]))/mSoftWorld->GetTimeStep();
	}
	// std::cout<<p.transpose()<<std::endl;
	// std::cout<<v.transpose()<<std::endl;
	// std::cout<<mBalls[mBallIndex]->GetVelocity().transpose()<<std::endl;
	mCount++;
	return std::string("no_event");
}





void
BezierCurveState::
GenerateMotions(const BezierCurve& bc,std::vector<std::pair<Eigen::VectorXd,double>>& motions)
{
	motions.clear();

	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
	Eigen::VectorXd save_positions = ik->GetSolution();

	auto save_target = ik->GetTargets();
	for(int i =0;i<mNumCurveSample+1;i++)
	{
		double tt = ((double)i)/((double)mNumCurveSample) * mD*mT;
		
		Eigen::Vector3d p_ee = bc.GetPosition(tt);

		ik->AddTargetPositions(mAnchorPoint,p_ee);
		mIKSolver->ReOptimizeTNLP(mIKOptimization);
		Eigen::VectorXd sol = ik->GetSolution();
		motions.push_back(std::make_pair(sol,tt));
	}

	for(auto& target : save_target)
		ik->AddTargetPositions(target.first,target.second);

	ik->SetSolution(save_positions);
}



void
Machine::
InitializeJugglingState(const std::vector<int>& V)
{
	int max_V = 0;
	for(int i=0;i<V.size();i++)
		if(max_V<V[i])
			max_V = V[i];

	mJugglingStates.resize(V.size() + max_V);

	for(int i =0;i<V.size();i++)
	{
		if(mJugglingStates[i].ball_index==-1)
		{
			mJugglingStates[i].isLeftHand = i%2;
			mJugglingStates[i].ball_index = i;
			mJugglingStates[i].V = V[i];

			mJugglingStates[i + V[i]].ball_index = i;
			mJugglingStates[i + V[i]].isLeftHand = !mJugglingStates[i].isLeftHand;
		}
		else
		{
			mJugglingStates[i].V = V[i];
			mJugglingStates[i + V[i]].ball_index = mJugglingStates[i].ball_index;
			mJugglingStates[i + V[i]].isLeftHand = !mJugglingStates[i].isLeftHand;
		}
	}

	for(auto& js : mJugglingStates)
	{
		std::cout<<(js.isLeftHand?"LEFT ":"RIGHT ");
		std::cout<<js.ball_index<<" ";
		std::cout<<js.V<<std::endl;
	}
	mJugglingFrame = 0;
}
Machine::
Machine(const dart::simulation::WorldPtr& rigid_world,
		const std::shared_ptr<FEM::World>& soft_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls,
		const std::shared_ptr<Controller>& controller,double dt)
	:mCurrentState(nullptr),mRigidWorld(rigid_world),mSoftWorld(soft_world),mMusculoSkeletalSystem(musculo_skeletal_system),mBalls(balls),mController(controller),mdt(dt)
{
	dart::math::seedRand();
	mIKOptimization = new IKOptimization(mMusculoSkeletalSystem->GetSkeleton());

	mIKSolver = new IpoptApplication();
	mIKSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	mIKSolver->Options()->SetStringValue("jac_c_constant", "yes");
	mIKSolver->Options()->SetStringValue("hessian_constant", "yes");
	mIKSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	mIKSolver->Options()->SetIntegerValue("print_level", 2);
	mIKSolver->Options()->SetIntegerValue("max_iter", 1000);
	mIKSolver->Options()->SetNumericValue("tol", 1e-4);

	mIKSolver->Initialize();
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));

	Eigen::Vector3d l_loc = mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandL")->getCOM();
	Eigen::Vector3d r_loc = mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR")->getCOM();

	ik->AddTargetPositions(std::make_pair(mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandL"),Eigen::Vector3d::Zero()),l_loc);
	ik->AddTargetPositions(std::make_pair(mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR"),Eigen::Vector3d::Zero()),r_loc);

	mIKSolver->OptimizeTNLP(mIKOptimization);


	// std::vector<int> V_list{5,3,1,5,3,1,5,3,1,5,3,1,5,3,1};
	std::vector<int> V_list{1,1,1,1};
// 	std::vector<int> V_list{3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
// 3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3};
	// std::vector<int> V_list{4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
	// std::vector<int> V_list{4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
	// std::vector<int> V_list{5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
	// std::vector<int> V_list{7,7,7,7,7,7,7,7,7};
	InitializeJugglingState(V_list);
}


State*
Machine::
AddState(const std::string& name)
{
	bool is_left =false;
	bool is_swing = false;
	if(name.find("LEFT")!=std::string::npos)
		is_left = true;
	if(name.find("SWING")!=std::string::npos)
		is_swing = true;
	if(is_swing)
			mStates.insert(std::make_pair(name,
		new BezierCurveState(
			mRigidWorld,
			mSoftWorld,
			mMusculoSkeletalSystem,
			mController,
			mMusculoSkeletalSystem->GetSkeleton()->getBodyNode((is_left?"HandL":"HandR")),
			mBalls,
			mIKOptimization,
			mIKSolver)));
	else
			mStates.insert(std::make_pair(name,
		new IKState(
			mRigidWorld,
			mSoftWorld,
			mMusculoSkeletalSystem,
			mController,
			mMusculoSkeletalSystem->GetSkeleton()->getBodyNode((is_left?"HandL":"HandR")),
			mBalls,
			mIKOptimization,
			mIKSolver)));
	
	if(mStates.size()==1)
		mCurrentState = mStates.at(name);
	
	return mStates.at(name);
}
void 	
Machine::
AddEvent(State* state_from,State* state_to,const std::string& name)
{
	state_from->AddEvent(name,state_to);
}

void
Machine::
SetCurrentState(State* s)
{
	mCurrentState = s;
}
State* 	
Machine::
GetCurrentState()
{
	return mCurrentState;
}
std::map<std::string,State*>&
Machine::
GetStates()
{
	return mStates;
}
void	
Machine::
Trigger(const std::string& name)
{
	std::string re_name = name;
	if(!name.compare("end"))
	{
		bool isPreviousLeft = mJugglingStates[mJugglingFrame].isLeftHand;
		// std::cout<<"isPreviousLeft : "<<isPreviousLeft<<std::endl;
		for(int i = mJugglingFrame+1;i<mJugglingStates.size();i++)
		{
			// std::cout<<"\nFrame : "<<i<<std::endl;
			// std::cout<<"isLeft : "<<mJugglingStates[i].isLeftHand<<std::endl;
			// std::cout<<"Ball index "<<mJugglingStates[i].ball_index<<std::endl;
			if(isPreviousLeft == mJugglingStates[i].isLeftHand)
			{
				if(mBalls[mJugglingStates[i].ball_index]->isReleased)
				{
					
					IKState* ikstate = dynamic_cast<IKState*>(mCurrentState->GetNextState(name +"_same_hand"));
					
					ikstate->Initialize(mJugglingStates[i].ball_index,mJugglingStates[i].V);
					
				}
				break;
			}
		}

		mJugglingFrame++;
		bool isCurrentLeft = mJugglingStates[mJugglingFrame].isLeftHand;
		if(isCurrentLeft == isPreviousLeft)
			re_name = name +"_same_hand";

	}
	State* next_state;
	next_state = mCurrentState->GetNextState(re_name);
	if(next_state!=nullptr){
		mCurrentState = next_state;
		mCurrentState->Initialize(mJugglingStates[mJugglingFrame].ball_index,mJugglingStates[mJugglingFrame].V);
	}	
	
}

void
Machine::
GetMotion(Eigen::VectorXd& p,Eigen::VectorXd& v)
{
	if(dynamic_cast<BezierCurveState*>(mCurrentState)!=nullptr)
	{
		if(dynamic_cast<BezierCurveState*>(mCurrentState)->mReleaseCount != 10000)
		{


		bool isCurrentLeft = mJugglingStates[mJugglingFrame].isLeftHand;
		for(int i = mJugglingFrame+1;i<mJugglingStates.size();i++)
		{
			bool isLeft = mJugglingStates[i].isLeftHand;
			if(isCurrentLeft == !isLeft)
			{
				IKState* ikstate = dynamic_cast<IKState*>(mCurrentState->GetNextState("end"));

				Eigen::Vector3d body_COM = ikstate->mAnchorPoint.first->getCOM();
				Eigen::Vector3d ball_COM= mBalls[mJugglingStates[i].ball_index]->GetPosition();;

				if((body_COM-ball_COM).norm()<5E-2){
					mBalls[mJugglingStates[i].ball_index]->Attach(mRigidWorld,ikstate->mAnchorPoint.first);
				}
				break;
			}
		}
		}
	}
	std::string event = mCurrentState->GetMotion(p,v);	
	mCurrentState->TimeStepping(mdt);
	Trigger(event);

}
void
MakeMachine(const std::string& file_path,const std::shared_ptr<Machine>& machine)
{
    TiXmlDocument doc;
    if(!doc.LoadFile(file_path))
    {
        std::cout<<"Cant open XML file : "<<file_path<<std::endl;
        return;
    }

    TiXmlElement* machine_xml = doc.FirstChildElement("Machine");

    for(TiXmlElement* state_xml = machine_xml->FirstChildElement("State");state_xml!=nullptr;state_xml = state_xml->NextSiblingElement("State"))
        machine->AddState(state_xml->Attribute("name"));
    auto& states = machine->GetStates();
    for(TiXmlElement* event_xml = machine_xml->FirstChildElement("Event");event_xml!=nullptr;event_xml = event_xml->NextSiblingElement("Event"))
        machine->AddEvent(states[event_xml->Attribute("from")],states[event_xml->Attribute("to")],event_xml->Attribute("name"));
}
