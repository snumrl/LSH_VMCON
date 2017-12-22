#ifndef __MUSCULO_SKELETAL_LQR_H__
#define __MUSCULO_SKELETAL_LQR_H__
#include "iLQR.h"
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include "../FSM/BezierCurve.h"
#include "fem/fem.h"
#include "dart/dart.hpp"
#include "dart/simulation/simulation.hpp"
class Ball;
class MusculoSkeletalSystem;
class MuscleOptimization;
class IKOptimization;

class MusculoSkeletalLQR : public iLQR
{
public:
	MusculoSkeletalLQR(
		const dart::simulation::WorldPtr& rigid_world,
		const std::shared_ptr<FEM::World>& soft_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls,int max_iteration);
	void Initialze(
		const Eigen::Vector3d& pos_desired,
		const Eigen::Vector3d& vel_desired,
		int index,int next_index, dart::dynamics::BodyNode* next_body,bool next_ball_initially_attached,
		const std::vector<Eigen::VectorXd>& reference_motions,
		const Eigen::VectorXd& x0,const std::vector<Eigen::VectorXd>& u0);

public:

	void EvalCf(const Eigen::VectorXd& x,double& cf) override;
	void EvalCfx(const Eigen::VectorXd& x,Eigen::VectorXd& cfx) override;
	void EvalCfxx(const Eigen::VectorXd& x,Eigen::MatrixXd& cfxx) override;

	void EvalC( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,double& c) override;
	void EvalCx( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& cx) override;
	void EvalCu( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& cu) override;
	void EvalCxx(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cxx) override;
	void EvalCxu(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cxu) override;
	void EvalCuu(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cuu) override;

	void Evalf(  const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& f) override;
	void Evalfx( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& fx) override;
	void Evalfu( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& fu) override;
	void Finalize(int& iteration) override;
	void WriteXML(const std::string& path);
	void WriteRecord(const std::string& path);
	int mWriteCount;
	std::string mWritePath;
	int mInitCount;
protected:
	void SetState(const Eigen::VectorXd& x,bool update_fem = true);
	void SetControl(const Eigen::VectorXd& u,double t);
	void GetState(Eigen::VectorXd& x);
	void Step();
	void ClipU(int i,double& lu,double& uu);
	void ClipX(int i,double& lx,double& ux);
	int 										mDofs;
	int 										mSoftWorldDofs;
	dart::simulation::WorldPtr 					mRigidWorld;
	std::shared_ptr<FEM::World>					mSoftWorld;
	std::shared_ptr<MusculoSkeletalSystem>		mMusculoSkeletalSystem;
	
	Eigen::VectorXd								mTargetPositions,mTargetVelocities;
	Eigen::VectorXd 							mKp,mKv;
	Ipopt::SmartPtr<Ipopt::TNLP> 			 	mMuscleOptimization;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> 	mMuscleOptimizationSolver;
	// Ipopt::SmartPtr<Ipopt::TNLP> 			 	mIKOptimization;
	// Ipopt::SmartPtr<Ipopt::IpoptApplication> 	mIKSolver;

	std::vector<Eigen::VectorXd>				mReferenceMotions;

	double 										w_regularization,w_smooth,w_compliance,w_pos_track,w_vel_track;

	std::vector<std::shared_ptr<Ball>>		 	mBalls;
	int 										mBallIndex;
	int 										mNextBallIndex;
	dart::dynamics::BodyNode* 					mNextBody;
	bool 										mNextBallInitiallyAttached;
	Eigen::Vector3d								mBallTargetPosition;
	Eigen::Vector3d								mBallTargetVelocity;
};


#endif