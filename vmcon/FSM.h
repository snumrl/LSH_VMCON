#ifndef __FSM_H__
#define __FSM_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "fem/fem.h"
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <vector>
#include <string>
#include <map>
#include <utility>
#include <Eigen/Core>
#include "BezierCurve.h"
class IKOptimization;
class MusculoSkeletalSystem;
class MusculoSkeletalLQR;
class Record;
class Controller;
class Ball;
typedef std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d> AnchorPoint;

struct JugglingState
{
	bool isLeftHand;  		// true : left hand, false : right hand
	int ball_index;
	int V;					//SiteSwap Value;

	JugglingState()
		:isLeftHand(false),ball_index(-1),V(0){};
};

class State
{
public:
	dart::simulation::WorldPtr							mRigidWorld;
	FEM::WorldPtr										mSoftWorld;
	std::shared_ptr<MusculoSkeletalSystem>				mMusculoSkeletalSystem;
	std::shared_ptr<Controller>							mController;

	AnchorPoint 										mAnchorPoint;

	std::vector<std::shared_ptr<Ball>>					mBalls;
	int 												mBallIndex;
	int 												mV;

	Ipopt::SmartPtr<Ipopt::TNLP>			 			mIKOptimization;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> 			mIKSolver;

	std::map<std::string,State*>						mEvents;
	double												mTimeElapsed;

public:
	State(	const dart::simulation::WorldPtr& rigid_world,
				const FEM::WorldPtr& soft_world,
				const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
				const std::shared_ptr<Controller>& controller,
				dart::dynamics::BodyNode* bn,
				const std::vector<std::shared_ptr<Ball>>& balls,
				const Ipopt::SmartPtr<Ipopt::TNLP>& ik_optimization,
				const Ipopt::SmartPtr<Ipopt::IpoptApplication>& ik_solver);

	void 	AddEvent(const std::string& name,State* next_state);
	State*	GetNextState(const std::string& event_name);
	std::map<std::string,State*>& GetEvents();

	
	void TimeStepping(double time_step);
	virtual void Initialize(int ball_index,int V) = 0; 
	double GetTime() {return mTimeElapsed;};
	virtual std::string GetMotion(Eigen::VectorXd& p,Eigen::VectorXd& v) = 0;
};

class IKState : public State
{
protected:
	void Solve();
public:
	IKState(	const dart::simulation::WorldPtr& rigid_world,
				const FEM::WorldPtr& soft_world,
				const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
				const std::shared_ptr<Controller>& controller,
				dart::dynamics::BodyNode* bn,
				const std::vector<std::shared_ptr<Ball>>& balls,
				const Ipopt::SmartPtr<Ipopt::TNLP>& ik_optimization,
				const Ipopt::SmartPtr<Ipopt::IpoptApplication>& ik_solver);

	void Initialize(int ball_index,int V) override;
	std::string GetMotion(Eigen::VectorXd& p,Eigen::VectorXd& v) override;
	Eigen::Vector3d GetTarget();
};

class BezierCurveState : public State
{

public:
	BezierCurveState(const dart::simulation::WorldPtr& rigid_world,
				const FEM::WorldPtr& soft_world,
				const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
				const std::shared_ptr<Controller>& controller,
				dart::dynamics::BodyNode* bn,
				const std::vector<std::shared_ptr<Ball>>& balls,
				const Ipopt::SmartPtr<Ipopt::TNLP>& ik_optimization,
				const Ipopt::SmartPtr<Ipopt::IpoptApplication>& ik_solver,
				double D = 0.7,
				double T = 0.3,
				int num_curve_sample = 10
				);

	void Initialize(int ball_index,int V) override;
	void InitializeLQR();
	void SynchronizeLQR();
	std::string GetMotion(Eigen::VectorXd& p,Eigen::VectorXd& v) override;
	const BezierCurve& GetCurve() {return mCurve;};

public:
	BezierCurve mCurve;
	// Eigen::Vector3d								   mTargetVelocity;
	std::vector<std::pair<Eigen::VectorXd,double>> mMotions;
	double mD,mT;
	int mNumCurveSample;
	int mReleaseCount;
	int mCount;

	FEM::WorldPtr 								mLQRSoftWorld;
	dart::simulation::WorldPtr  				mLQRRigidWorld;
	std::shared_ptr<MusculoSkeletalSystem>		mLQRMusculoSkeletalSystem;
	std::vector<std::shared_ptr<Ball>>			mLQRBalls;

	std::shared_ptr<MusculoSkeletalLQR>			mLQR;
	std::vector<Eigen::VectorXd> 				mU;

	void GenerateMotions(const BezierCurve& bc,std::vector<std::pair<Eigen::VectorXd,double>>& motions);
	void OptimizeLQR(const Eigen::Vector3d& p_des,const Eigen::Vector3d& v_des);
};




class Machine
{
private:
	State*										mCurrentState;

	std::map<std::string,State*>				mStates;

	dart::simulation::WorldPtr					mRigidWorld;
	FEM::WorldPtr								mSoftWorld;
	std::shared_ptr<MusculoSkeletalSystem>		mMusculoSkeletalSystem;
	
	std::vector<std::shared_ptr<Ball>>			mBalls;
	std::shared_ptr<Controller>					mController;

	Ipopt::SmartPtr<Ipopt::TNLP>			 	mIKOptimization;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> 	mIKSolver;

	std::vector<JugglingState>					mJugglingStates;
	int 										mJugglingFrame;

	double										mdt;
	void InitializeJugglingState(const std::vector<int>& V);
public:
	Machine(
		const dart::simulation::WorldPtr& rigid_world,
		const FEM::WorldPtr& soft_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls,
		const std::shared_ptr<Controller>& controller,double dt);
	
	State* 	AddState(const std::string& name);
	void 	AddEvent(State* state_from,State* state_to,const std::string& name);

	void 	SetCurrentState(State* s);
	State* 	GetCurrentState();
	std::map<std::string,State*>& GetStates();
	void	Trigger(const std::string& event_name);
	void GetMotion(Eigen::VectorXd& p,Eigen::VectorXd& v);

};


void MakeMachine(const std::string& file_path,const std::shared_ptr<Machine>& machine);
#endif