#include "FSM.h"
#include "Ball.h"
#include "Record.h"
#include "BezierCurve.h"
#include "IKOptimization.h"
#include "MusculoSkeletalSystem.h"
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
	bool need_ik_update= true;
	
	Eigen::Vector3d target;
	mBalls[mBallIndex]->ComputeFallingPosition(mBalls[mBallIndex]->releasedPoint[1],target);
	
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
	
	const auto& ik_targets = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization))->GetTargets();
	for(auto& t : ik_targets)
	{
		if(!t.first.first->getName().compare(mAnchorPoint.first->getName()))
		{
			if((t.second-target).norm()<5E-3)
				need_ik_update = false;
		}
	}
	if(need_ik_update)
	{
		ik->AddTargetPositions(mAnchorPoint,target);
		mIKSolver->ReOptimizeTNLP(mIKOptimization);
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
		event = "catch";
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
}

void 
BezierCurveState::
Initialize(int ball_index,int V)
{
	mTimeElapsed = 0;
	mBallIndex = ball_index;
	mV = V;
	double t = ((double)mV-2*mD+0.3)*mT;

	Eigen::Vector3d p0,p1,p2,v2;
	p0.setZero();
	p1.setZero();
	p2.setZero();
	v2.setZero();

	
	bool isleft = false;
	if(mAnchorPoint.first->getName().find("L")!=std::string::npos)
		isleft = true;

	p0 = mBalls[mBallIndex]->GetPosition();
	
	if(isleft){
		p0[0] +=0.1;
		p2[0] = p0[0]-0.15;
	}
	else{
		p0[0] -=0.1;
		p2[0] = p0[0]+0.15;
	}
	p2[1] = 0.1;
	p2[2] = p0[2];

	

	v2[0] = -2.0*p0[0]/t;
	// v2[0] = 0;
	v2[1] = 0.5*9.81*t;
	v2[2] = -(p0[2]-0.3)/t;
	
	// p0[2] = 0.3;
	p1 = p2 - 0.5*mD*mT*v2;

	std::cout<<"p0 : "<<p0.transpose()<<std::endl;
	std::cout<<"p1 : "<<p1.transpose()<<std::endl;
	std::cout<<"p2 : "<<p2.transpose()<<std::endl;
	std::cout<<"v2 : "<<v2.transpose()<<std::endl;

	mCurve.Initialize(p0,p1,p2,mD*mT);
	GenerateMotions(mCurve,mMotions);
}
std::string 
BezierCurveState::
GetMotion(Eigen::VectorXd& p,Eigen::VectorXd& v)
{
	int k =0,k1 =0;
	for(int i =0;i<mMotions.size();i++)
	{
		if(mMotions[i].second<mTimeElapsed)
			k=i;
	}

	
	if(k == mMotions.size()-1)
	{
		mBalls[mBallIndex]->Release(mRigidWorld);

		return std::string("end");
	}

	k1 = k+1;
	double t = mTimeElapsed-mMotions[k].second;
	double dt = mMotions[k1].second-mMotions[k].second;

	t/= dt;
	p = (1.0-t)*(mMotions[k].first) + (t)*(mMotions[k1].first);

	double t1 = t+0.01;
	auto pdp =(1.0-t)*(mMotions[k].first) + (t)*(mMotions[k1].first);

	v = (pdp-p)/0.01;
	mCount++;
	return std::string("no_event");
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
	std::vector<int> V_list{3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3};
	// std::vector<int> V_list{4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
	// std::vector<int> V_list{4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
	// std::vector<int> V_list{5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
	// std::vector<int> V_list{7,7,7,7,7,7,7,7,7};
	InitializeJugglingState(V_list);
}


State*
Machine::
AddState(const std::string& name)
{
	bool is_left =false;
	bool is_catch = false;
	if(name.find("LEFT")!=std::string::npos)
		is_left = true;
	if(name.find("CATCH")!=std::string::npos)
		is_catch = true;
	if(is_catch)
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
	else
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
	mCurrentState->TimeStepping(mdt);
	std::string event = mCurrentState->GetMotion(p,v);	
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