#ifndef __MUSCULO_SKELETAL_LQR_H__
#define __MUSCULO_SKELETAL_LQR_H__
#include "iLQR.h"
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include "fem2D/World.h"
#include "dart/dart.hpp"
#include "dart/simulation/simulation.hpp"
class MusculoSkeletalSystem;
class MuscleOptimization;
class IKOptimization;

class MusculoSkeletalLQR : public iLQR
{
public:
	MusculoSkeletalLQR(
		const Eigen::Vector3d& pos_desired,
		const Eigen::Vector3d& vel_desired,
		const dart::simulation::WorldPtr& rigid_world,
		FEM::World* soft_world,
		MusculoSkeletalSystem* musculo_skeletal_system,int n,int max_iteration);
	void Initialze(
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

protected:
	void SetState(const Eigen::VectorXd& x);
	void SetControl(const Eigen::VectorXd& u,double t);
	void GetState(Eigen::VectorXd& x);
	void Step();
	void ClipU(int i,double& lu,double& uu);
	void ClipX(int i,double& lx,double& ux);
	int 										mDofs;
	dart::simulation::WorldPtr 					mRigidWorld;
	FEM::World*									mSoftWorld;
	MusculoSkeletalSystem* 						mMusculoSkeletalSystem;

	Eigen::VectorXd								mTargetPositions,mTargetVelocities;
	Eigen::VectorXd 							mKp,mKv;
	Ipopt::SmartPtr<Ipopt::TNLP> 			 	mMuscleOptimization;
	Ipopt::SmartPtr<Ipopt::IpoptApplication> 	mMuscleOptimizationSolver;
	// Ipopt::SmartPtr<Ipopt::TNLP> 			 	mIKOptimization;
	// Ipopt::SmartPtr<Ipopt::IpoptApplication> 	mIKSolver;

	Eigen::VectorXd								mSoftWorldX0;
	std::vector<Eigen::VectorXd>				mInitialGuess;

	double 										w_regularization,w_compliance,w_pos_track,w_vel_track;
	dart::dynamics::BodyNode* 					mEndEffector;
	Eigen::Vector3d								mEndEffectorTargetPosition;
	Eigen::Vector3d								mEndEffectorTargetVelocity;
};


#endif