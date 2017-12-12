#include "MusculoSkeletalLQR.h"
#include "../MusculoSkeletalSystem.h"
#include "../MuscleOptimization.h"
#include "../IKOptimization.h"
#include <fstream>
using namespace Ipopt;
MusculoSkeletalLQR::
MusculoSkeletalLQR(
		const Eigen::Vector3d& pos_desired,
		const Eigen::Vector3d& vel_desired,
		const dart::simulation::WorldPtr& rigid_world,
		FEM::World* soft_world,
		MusculoSkeletalSystem* musculo_skeletal_system,int n,int max_iteration)
		:iLQR(	musculo_skeletal_system->GetSkeleton()->getNumDofs()*2, 			//State
				musculo_skeletal_system->GetSkeleton()->getNumDofs(),			//Signal
				n,max_iteration),
		mEndEffectorTargetPosition(pos_desired),
		mEndEffectorTargetVelocity(vel_desired),
		mDofs(musculo_skeletal_system->GetSkeleton()->getNumDofs()),
		mRigidWorld(rigid_world),mSoftWorld(soft_world),mMusculoSkeletalSystem(musculo_skeletal_system),
		mTargetPositions(Eigen::VectorXd::Zero(musculo_skeletal_system->GetSkeleton()->getNumDofs())),
		mTargetVelocities(Eigen::VectorXd::Zero(musculo_skeletal_system->GetSkeleton()->getNumDofs())),
		mKp(Eigen::VectorXd::Constant(musculo_skeletal_system->GetSkeleton()->getNumDofs(),1000.0)),
		mKv(Eigen::VectorXd::Constant(musculo_skeletal_system->GetSkeleton()->getNumDofs(),2*sqrt(1000.0))),
		mSoftWorldX0(mSoftWorld->GetPositions())
{
	mMuscleOptimization = new MuscleOptimization(mSoftWorld,mRigidWorld,mMusculoSkeletalSystem);
	mMuscleOptimizationSolver = new IpoptApplication();
	
	mMuscleOptimizationSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	mMuscleOptimizationSolver->Options()->SetStringValue("jac_c_constant", "no");
	mMuscleOptimizationSolver->Options()->SetStringValue("hessian_constant", "yes");
	mMuscleOptimizationSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	mMuscleOptimizationSolver->Options()->SetIntegerValue("print_level", 2);
	mMuscleOptimizationSolver->Options()->SetIntegerValue("max_iter", 100);
	mMuscleOptimizationSolver->Options()->SetNumericValue("tol", 1e-4);

	mMuscleOptimizationSolver->Initialize();
	mMuscleOptimizationSolver->OptimizeTNLP(mMuscleOptimization);

	// mIKOptimization = new IKOptimization(mMusculoSkeletalSystem->GetSkeleton());

	// mIKSolver = new IpoptApplication();
	// mIKSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	// mIKSolver->Options()->SetStringValue("jac_c_constant", "yes");
	// mIKSolver->Options()->SetStringValue("hessian_constant", "yes");
	// mIKSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	// mIKSolver->Options()->SetIntegerValue("print_level", 2);
	// mIKSolver->Options()->SetIntegerValue("max_iter", 10);
	// mIKSolver->Options()->SetNumericValue("tol", 1e-4);

	// mIKSolver->Initialize();

	mEndEffector = mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR");
	std::ifstream param("../vmcon2D/export/param.txt");
	param>>w_regularization>>w_compliance>>w_pos_track>>w_vel_track;
	param.close();
}
void
MusculoSkeletalLQR::
Initialze(
	const Eigen::VectorXd& x0,const std::vector<Eigen::VectorXd>& u0)
{
	mInitialGuess = u0;
	Eigen::VectorXd u_lower(mMusculoSkeletalSystem->GetSkeleton()->getNumDofs());
	Eigen::VectorXd u_upper(mMusculoSkeletalSystem->GetSkeleton()->getNumDofs());
	for(int i =0;i<mMusculoSkeletalSystem->GetSkeleton()->getNumDofs();i++)
	{
		u_lower[i] = mMusculoSkeletalSystem->GetSkeleton()->getDof(i)->getPositionLowerLimit();
		u_upper[i] = mMusculoSkeletalSystem->GetSkeleton()->getDof(i)->getPositionUpperLimit();
	}
	// u_lower[mDofs] = 0.5;
	// u_upper[mDofs] = 2.0;
	Init(x0,u0,u_lower,u_upper);
}
void
MusculoSkeletalLQR::
ClipU(int i,double& lu,double& uu)
{
	// if(uu>mu_upper[i])
	// 	uu = mu_upper[i];
	// if(lu<mu_lower[i])
	// 	lu = mu_lower[i];
}
void
MusculoSkeletalLQR::
ClipX(int i,double& lx,double& ux)
{
	// int n = mMusculoSkeletalSystem->GetSkeleton()->getNumDofs();
	// if(i<n)
	// {
	// 	if(ux>mu_upper[i])
	// 		ux = mu_upper[i];
	// 	if(lx<mu_lower[i])
	// 		lx = mu_lower[i];
	// }
}

void
MusculoSkeletalLQR::
EvalCf(const Eigen::VectorXd& x,double& cf)
{
	SetState(x);

	Eigen::Vector3d ee_pos = mEndEffector->getCOM();
	Eigen::Vector3d ee_vel = mEndEffector->getCOMLinearVelocity();

	cf = 0.5*w_pos_track*(mEndEffectorTargetPosition - ee_pos).squaredNorm();
	cf += 0.5*w_vel_track*(mEndEffectorTargetVelocity - ee_vel).squaredNorm();
}

void
MusculoSkeletalLQR::
EvalC( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,double& c)
{
	// if(t == 0)
		// c = 0.5*w_smooth*(u-mInitialGuess[0]).squaredNorm();
	// else
	c = 0.5*w_regularization*((u-mInitialGuess[t]).head(mDofs)).squaredNorm();
	// c += 0.5*w_compliance*(u[mDofs]*u[mDofs]);
	// SetState(x);
	
	// Eigen::Vector3d ee_pos = mEndEffector->getTransform()*Eigen::Vector3d(0,0,0);
	// c += 1E-2*(double)t/(double)mN*0.5*w_tracking*(mEndEffectorTarget - ee_pos).squaredNorm();
	// c += 1E-2*(double)t/(double)mN*0.5*w_stable*(mEndEffector->getCOMLinearVelocity()).squaredNorm();	
}

void
MusculoSkeletalLQR::
Evalf(  const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& f)
{
	if(t==0)
		mSoftWorld->SetPositions(mSoftWorldX0);

	SetState(x);
	SetControl(u,t);
	Step();
	GetState(f);
}

void
MusculoSkeletalLQR::
SetState(const Eigen::VectorXd& x)
{
	mMusculoSkeletalSystem->GetSkeleton()->setPositions(x.head(mDofs));
	mMusculoSkeletalSystem->GetSkeleton()->setVelocities(x.tail(mDofs));
	mMusculoSkeletalSystem->GetSkeleton()->computeForwardKinematics(true,false,false);
}
void
MusculoSkeletalLQR::
SetControl(const Eigen::VectorXd& u,double t)
{
	mTargetPositions = u.head(mDofs);
	if(t == 0)
		mTargetVelocities.setZero();
	else
		mTargetVelocities = (u.head(mDofs)-mu[t-1].head(mDofs))/mSoftWorld->GetTimeStep();

	// mKp = Eigen::VectorXd::Constant(mDofs,1000.0*u[mDofs]);
	// mKv = Eigen::VectorXd::Constant(mDofs,2*sqrt(mKp[0]));
}
void
MusculoSkeletalLQR::
GetState(Eigen::VectorXd& x)
{
	x.head(mDofs) = mMusculoSkeletalSystem->GetSkeleton()->getPositions();
	x.tail(mDofs) = mMusculoSkeletalSystem->GetSkeleton()->getVelocities();
}
void
MusculoSkeletalLQR::
Step()
{
	auto& skel = mMusculoSkeletalSystem->GetSkeleton();
	Eigen::VectorXd pos_diff = skel->getPositionDifferences(mTargetPositions,skel->getPositions());
	for(int i = 0;i<pos_diff.rows();i++)
		pos_diff[i] = dart::math::wrapToPi(pos_diff[i]);
	Eigen::VectorXd qdd_desired = pos_diff.cwiseProduct(mKp) + (mTargetVelocities - skel->getVelocities()).cwiseProduct(mKv);
	static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->Update(qdd_desired);
	mMuscleOptimizationSolver->ReOptimizeTNLP(mMuscleOptimization);

	Eigen::VectorXd solution =  static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->GetSolution();

	mMusculoSkeletalSystem->SetActivationLevel(solution.tail(mMusculoSkeletalSystem->GetNumMuscles()));
	mMusculoSkeletalSystem->TransformAttachmentPoints();
	mSoftWorld->TimeStepping(false);

	double nn = mSoftWorld->GetTimeStep() / mRigidWorld->getTimeStep();
	for(int i =0; i<nn;i++)
	{
		mMusculoSkeletalSystem->ApplyForcesToSkeletons(mSoftWorld);
		// mMusculoSkeletalSystem->GetSkeleton()->setForces(
		// 	mMusculoSkeletalSystem->GetSkeleton()->getMassMatrix()*qdd_desired+
			// mMusculoSkeletalSystem->GetSkeleton()->getCoriolisAndGravityForces());
		mRigidWorld->step();
	}
}

































void
MusculoSkeletalLQR::
EvalCfx(const Eigen::VectorXd& x,Eigen::VectorXd& cfx)
{
	cfx.setZero();

	Eigen::VectorXd x_i;
	double delta = 0.01;
	double cf_minus,cf_plus;
	double x_i_plus,x_i_minus;
	for(int i = 0;i<mSx;i++)
	{
		x_i = x;
		cf_minus =0;
		cf_plus =0;
		x_i_minus = x[i] - delta;
		x_i_plus = x[i] + delta;
	
		ClipX(i,x_i_minus,x_i_plus);
		
		x_i[i] = x_i_minus;
		EvalCf(x_i,cf_minus);
		x_i[i] = x_i_plus;
		EvalCf(x_i,cf_plus);

		cfx[i] = (cf_plus - cf_minus)/(x_i_plus - x_i_minus);
	}
}
void
MusculoSkeletalLQR::
EvalCfxx(const Eigen::VectorXd& x,Eigen::MatrixXd& cfxx)
{
	cfxx.resize(mSx,mSx);
	cfxx.setZero();
	Eigen::VectorXd x_i;
	double delta = 0.01;
	Eigen::VectorXd cfx_minus(mSx),cfx_plus(mSx);
	double x_i_minus,x_i_plus;
	for(int i = 0;i<mSx;i++)
	{
		x_i = x;
		cfx_minus.setZero();
		cfx_plus.setZero();
		x_i_minus = x[i] - delta;
		x_i_plus = x[i] + delta;

		ClipX(i,x_i_minus,x_i_plus);

		x_i[i] = x_i_minus;
		EvalCfx(x_i,cfx_minus);
		x_i[i] = x_i_plus;
		EvalCfx(x_i,cfx_plus);

		cfxx.col(i) = (cfx_plus - cfx_minus)/(x_i_plus - x_i_minus);
	}
}

void
MusculoSkeletalLQR::
EvalCx( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& cx)
{
	cx.setZero();

	Eigen::VectorXd x_i;
	double delta = 0.01;
	double c_minus,c_plus;
	double x_i_plus,x_i_minus;
	for(int i = 0;i<mSx;i++)
	{
		x_i = x;
		c_minus =0;
		c_plus =0;
		x_i_minus = x[i] - delta;
		x_i_plus = x[i] + delta;
	
		ClipX(i,x_i_minus,x_i_plus);
		
		x_i[i] = x_i_minus;
		EvalC(x_i,u,t,c_minus);
		x_i[i] = x_i_plus;
		EvalC(x_i,u,t,c_plus);

		cx[i] = (c_plus - c_minus)/(x_i_plus - x_i_minus);
	}
}
void
MusculoSkeletalLQR::
EvalCu( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& cu)
{
	cu.setZero();

	Eigen::VectorXd u_i;
	double delta = 0.01;
	double c_minus,c_plus;
	double u_i_plus,u_i_minus;
	for(int i = 0;i<mSu;i++)
	{
		u_i = u;
		c_minus =0;
		c_plus =0;
		u_i_minus = u[i] - delta;
		u_i_plus = u[i] + delta;
	
		ClipU(i,u_i_minus,u_i_plus);
		
		u_i[i] = u_i_minus;
		EvalC(x,u_i,t,c_minus);
		u_i[i] = u_i_plus;
		EvalC(x,u_i,t,c_plus);

		cu[i] = (c_plus - c_minus)/(u_i_plus - u_i_minus);
	}
}
void
MusculoSkeletalLQR::
EvalCxx(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cxx)
{
	cxx.resize(mSx,mSx);
	cxx.setZero();
	Eigen::VectorXd x_i;
	double delta = 0.01;
	Eigen::VectorXd cx_minus(mSx),cx_plus(mSx);
	double x_i_minus,x_i_plus;
	for(int i = 0;i<mSx;i++)
	{
		x_i = x;
		cx_minus.setZero();
		cx_plus.setZero();
		x_i_minus = x[i] - delta;
		x_i_plus = x[i] + delta;

		ClipX(i,x_i_minus,x_i_plus);

		x_i[i] = x_i_minus;
		EvalCx(x_i,u,t,cx_minus);
		x_i[i] = x_i_plus;
		EvalCx(x_i,u,t,cx_plus);

		cxx.col(i) = (cx_plus - cx_minus)/(x_i_plus - x_i_minus);
	}
}
void
MusculoSkeletalLQR::
EvalCxu(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cxu)
{
	cxu.resize(mSx,mSu);
	cxu.setZero();
	Eigen::VectorXd u_i;
	double delta = 0.01;
	Eigen::VectorXd cx_minus(mSx),cx_plus(mSx);
	double u_i_minus,u_i_plus;
	for(int i = 0;i<mSu;i++)
	{
		u_i = u;
		cx_minus.setZero();
		cx_plus.setZero();
		u_i_minus = u[i] - delta;
		u_i_plus = u[i] + delta;
	
		ClipU(i,u_i_minus,u_i_plus);

		u_i[i] = u_i_minus;
		EvalCx(x,u_i,t,cx_minus);
		u_i[i] = u_i_plus;
		EvalCx(x,u_i,t,cx_plus);

		cxu.col(i) = (cx_plus - cx_minus)/(u_i_plus - u_i_minus);
	}
}
void
MusculoSkeletalLQR::
EvalCuu(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cuu)
{
	cuu.resize(mSu,mSu);
	cuu.setZero();
	Eigen::VectorXd u_i;
	double delta = 0.01;
	Eigen::VectorXd cu_minus(mSu),cu_plus(mSu);
	double u_i_minus,u_i_plus;

	for(int i = 0;i<mSu;i++)
	{
		u_i = u;
		cu_minus.setZero();
		cu_plus.setZero();
		u_i_minus = u[i] - delta;
		u_i_plus = u[i] + delta;
	
		ClipU(i,u_i_minus,u_i_plus);

		u_i[i] = u_i_minus;
		EvalCu(x,u_i,t,cu_minus);
		u_i[i] = u_i_plus;
		EvalCu(x,u_i,t,cu_plus);

		cuu.col(i) = (cu_plus - cu_minus)/(u_i_plus - u_i_minus);
	}
}



void
MusculoSkeletalLQR::
Evalfx( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& fx)
{
	fx.resize(mSx,mSx);
	fx.setZero();

	Eigen::VectorXd x_i;
	double delta = 0.01;
	Eigen::VectorXd fx_i_minus(mSx),fx_i_plus(mSx);
	double x_i_minus,x_i_plus;
	for(int i = 0;i<mSx;i++)
	{
		x_i = x;

		fx_i_minus.setZero();
		fx_i_plus.setZero();

		x_i_minus = x[i] - delta;
		x_i_plus = x[i] + delta;

		ClipX(i,x_i_minus,x_i_plus);

		x_i[i] = x_i_minus;
		Evalf(x_i,u,t,fx_i_minus);
		x_i[i] = x_i_plus;
		Evalf(x_i,u,t,fx_i_plus);

		fx.col(i) = (fx_i_plus - fx_i_minus)/(x_i_plus - x_i_minus);
	}
}
void
MusculoSkeletalLQR::
Evalfu( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& fu)
{
	fu.resize(mSx,mSu);
	fu.setZero();

	Eigen::VectorXd u_i;
	double delta = 0.01;
	Eigen::VectorXd fu_i_minus(mSx),fu_i_plus(mSx);
	double u_i_minus,u_i_plus;
	for(int i = 0;i<mSu;i++)
	{
		u_i = u;

		fu_i_minus.setZero();
		fu_i_plus.setZero();

		u_i_minus = u[i] - delta;
		u_i_plus = u[i] + delta;

		ClipU(i,u_i_minus,u_i_plus);

		u_i[i] = u_i_minus;
		Evalf(x,u_i,t,fu_i_minus);
		u_i[i] = u_i_plus;
		Evalf(x,u_i,t,fu_i_plus);

		fu.col(i) = (fu_i_plus - fu_i_minus)/(u_i_plus - u_i_minus);
	}
}
