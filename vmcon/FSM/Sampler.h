#ifndef __SAMPLER_H__
#define __SAMPLER_H__
#include "Machine.h"
#include <memory>
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "fem/fem.h"
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
class Ball;
class MusculoSkeletalSystem;
class BezierCurve;
typedef std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d> AnchorPoint;

class Sampler
{
public:
Sampler(
		const dart::simulation::WorldPtr& rigid_world,
		const std::shared_ptr<FEM::World>& soft_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls);

	void Initialze(
		const Eigen::Vector3d& pos_desired,
		const Eigen::Vector3d& vel_desired,
		int index, dart::dynamics::BodyNode* body,const std::vector<std::pair<AnchorPoint,Eigen::Vector3d>>& ik_targets,
		int next_index, dart::dynamics::BodyNode* next_body,bool next_ball_initially_attached,const Eigen::VectorXd& s0,double t_hold);

	Eigen::VectorXd Solve(const Eigen::VectorXd& x0);
	double EvalC(const Eigen::VectorXd& x);
	Eigen::VectorXd EvalCx(const Eigen::VectorXd& x);
	Eigen::MatrixXd EvalCxx(const Eigen::VectorXd& x);

	void GenerateMotions(const Eigen::VectorXd& x,std::vector<Eigen::VectorXd>& motions);
	void SetS0();
	void Step(const Eigen::VectorXd& q,const Eigen::VectorXd& qd);
	void Simulate(const std::vector<Eigen::VectorXd>& motions);

	void Finalize(const Eigen::VectorXd& x_star,const std::string& post_index = "");
	void WriteXML(const std::string& path);
	void WriteRecord(const std::string& path);
	int mWriteCount;
	std::string mWritePath;
	int mInitCount;
public:
	Eigen::VectorXd								mS0;

	dart::simulation::WorldPtr 					mRigidWorld;
	std::shared_ptr<FEM::World>					mSoftWorld;
	std::shared_ptr<MusculoSkeletalSystem>		mMusculoSkeletalSystem;

	std::vector<Eigen::VectorXd>				mReferenceMotions;

	Eigen::VectorXd								mTargetPositions,mTargetVelocities;
	Eigen::VectorXd 							mKp,mKv;
	Ipopt::SmartPtr<Ipopt::TNLP> 			 	mMuscleOptimization;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> 	mMuscleOptimizationSolver;
	Ipopt::SmartPtr<Ipopt::TNLP>			 	mIKOptimization;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> 	mIKSolver;

	double 										w_regularization,w_smooth,w_compliance,w_pos_track,w_vel_track;
	
	double										mTHold;
	std::vector<std::shared_ptr<Ball>>		 	mBalls;
	int 										mBallIndex;
	dart::dynamics::BodyNode* 					mBody;
	int 										mNextBallIndex;
	dart::dynamics::BodyNode* 					mNextBody;
	bool 										mNextBallInitiallyAttached;

	Eigen::Vector3d								mBallTargetPosition;
	Eigen::Vector3d								mBallTargetVelocity;
};

#endif