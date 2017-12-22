#ifndef __MACHINE_H__
#define __MACHINE_H__
#include "Juggling.h"
#include <memory>
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "fem/fem.h"
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

class MusculoSkeletalLQR;
class Ball;
class MusculoSkeletalSystem;
class BezierCurve;
typedef std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d> AnchorPoint;

class Machine
{
public:

	Machine(
		const dart::simulation::WorldPtr& rigid_world,
		const FEM::WorldPtr& soft_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls,
		const std::vector<int>& sequences,int ball_size);

	void GetMotion(Eigen::VectorXd& p,Eigen::VectorXd& v);
	
public:
	void GenerateCatchMotions();
	void GenerateSwingMotions();
	std::vector<std::pair<Eigen::VectorXd,double>> GenerateFineMotions(const std::vector<std::pair<Eigen::VectorXd,double>>& coarse_motions);
	
	void InitializeLQR();
	void SynchronizeLQR();
	void OptimizeLQR(BezierCurve* initial_curve, const Eigen::Vector3d& p_des,const Eigen::Vector3d& v_des);
//For Juggling
	std::shared_ptr<JugglingInfo> 				mJugglingInfo;
	double										mTimeElapsed;
	int 										mCount;

	int 										mPhase; // 0: catch, 1:swing
	// bool 										mIsCatched; //For V ==1 case
	std::vector<std::pair<Eigen::VectorXd,double>> mMotions;
//For control
	dart::simulation::WorldPtr					mRigidWorld;
	FEM::WorldPtr								mSoftWorld;
	std::shared_ptr<MusculoSkeletalSystem>		mMusculoSkeletalSystem;
	std::vector<std::shared_ptr<Ball>>			mBalls;

//For Swing Phase
	BezierCurve* mCurve;

	FEM::WorldPtr 								mLQRSoftWorld;
	dart::simulation::WorldPtr  				mLQRRigidWorld;
	std::shared_ptr<MusculoSkeletalSystem>		mLQRMusculoSkeletalSystem;
	std::vector<std::shared_ptr<Ball>>			mLQRBalls;
	
	std::shared_ptr<MusculoSkeletalLQR>			mLQR;
	std::vector<Eigen::VectorXd> 				mU;

//For Catch Phase
	Ipopt::SmartPtr<Ipopt::TNLP>			 	mIKOptimization;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> 	mIKSolver;

	Eigen::Vector3d								mLocalOffset;
	Eigen::Vector3d								mHandX0;
};

#endif