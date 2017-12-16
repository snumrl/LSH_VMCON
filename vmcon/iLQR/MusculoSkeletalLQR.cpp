#include "MusculoSkeletalLQR.h"
#include "../MusculoSkeletalSystem.h"
#include "../MuscleOptimization.h"
#include "../IKOptimization.h"
#include "../Ball.h"

#include <fstream>
using namespace Ipopt;
MusculoSkeletalLQR::
MusculoSkeletalLQR(
		const dart::simulation::WorldPtr& rigid_world,
		const std::shared_ptr<FEM::World>& soft_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls,int max_iteration)
		:iLQR(	musculo_skeletal_system->GetSkeleton()->getNumDofs()*2+6*balls.size(), 			//State
				musculo_skeletal_system->GetSkeleton()->getNumDofs(),			//Signal
				max_iteration),
		mDofs(musculo_skeletal_system->GetSkeleton()->getNumDofs()),
		mRigidWorld(rigid_world),mSoftWorld(soft_world),mMusculoSkeletalSystem(musculo_skeletal_system),mBalls(balls),
		mTargetPositions(Eigen::VectorXd::Zero(musculo_skeletal_system->GetSkeleton()->getNumDofs())),
		mTargetVelocities(Eigen::VectorXd::Zero(musculo_skeletal_system->GetSkeleton()->getNumDofs())),
		mKp(Eigen::VectorXd::Constant(musculo_skeletal_system->GetSkeleton()->getNumDofs(),300.0)),
		mKv(Eigen::VectorXd::Constant(musculo_skeletal_system->GetSkeleton()->getNumDofs(),2*sqrt(300.0))),
		mSoftWorldX0(mSoftWorld->GetPositions())
{
	mKp[mDofs-2] = 2.0*mKp[mDofs-2];
	mKp[mDofs-1] = 2.0*mKp[mDofs-1];
	mKv[mDofs-2] = sqrt(2.0)*mKv[mDofs-1];
	mKv[mDofs-1] = sqrt(2.0)*mKv[mDofs-1];
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

	// mEndEffector = mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR");
	std::ifstream param("../vmcon/export/param.txt");
	param>>w_regularization>>w_smooth>>w_pos_track>>w_vel_track;
	param.close();
}
void
MusculoSkeletalLQR::
Initialze(
	const Eigen::Vector3d& pos_desired,
	const Eigen::Vector3d& vel_desired,
	int index,
	const std::vector<Eigen::VectorXd>& reference_motions,
	const Eigen::VectorXd& x0,const std::vector<Eigen::VectorXd>& u0)
{
	mBallIndex = index;
	std::cout<<"p_t : "<<pos_desired.transpose()<<std::endl;
	std::cout<<"v_t : "<<vel_desired.transpose()<<std::endl;
	mBallTargetPosition = pos_desired;
	mBallTargetVelocity = vel_desired;
	mReferenceMotions = reference_motions;
	Eigen::VectorXd u_lower(mMusculoSkeletalSystem->GetSkeleton()->getNumDofs());
	Eigen::VectorXd u_upper(mMusculoSkeletalSystem->GetSkeleton()->getNumDofs());
	for(int i =0;i<mMusculoSkeletalSystem->GetSkeleton()->getNumDofs();i++)
	{
		u_lower[i] = -1;//mMusculoSkeletalSystem->GetSkeleton()->getDof(i)->getPositionLowerLimit();
		u_upper[i] = 1;//mMusculoSkeletalSystem->GetSkeleton()->getDof(i)->getPositionUpperLimit();
	}
	// u_lower[mDofs] = 0.5;
	// u_upper[mDofs] = 2.0;
	// std::cout<<reference_motions.size()<<std::endl;
	Init(reference_motions.size(),x0,u0,u_lower,u_upper);
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
bool is_ioing = false;

void
MusculoSkeletalLQR::
EvalCf(const Eigen::VectorXd& x,double& cf)
{
	SetState(x);

	Eigen::Vector3d ball_pos = mBalls[mBallIndex]->GetPosition();
	Eigen::Vector3d ball_vel = mBalls[mBallIndex]->GetVelocity();

	if(is_ioing)
	{
		// std::cout<<"POS : "<<0.5*w_pos_track*(mBallTargetPosition - ball_pos).squaredNorm()<<std::endl;
		// std::cout<<"VEL : "<<0.5*w_vel_track*(mBallTargetVelocity - ball_vel).squaredNorm()<<std::endl;
		// std::cout<<"VEL : "<<(mBallTargetVelocity - ball_vel).transpose()<<std::endl;
	}
	// std::cout<<ball_vel.transpose()<<std::endl;
	cf = 0.5*w_pos_track*(mBallTargetPosition - ball_pos).squaredNorm();
	cf += 0.5*w_vel_track*(mBallTargetVelocity - ball_vel).squaredNorm();
}

void
MusculoSkeletalLQR::
EvalC( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,double& c)
{
	c = 0.5*w_regularization*(u).squaredNorm();
	if(t==0)
		c+= 0.5*w_smooth*(u).squaredNorm();
	else
		c+= 0.5*w_smooth*(u-mu[t-1]).squaredNorm();
}

void
MusculoSkeletalLQR::
Evalf(  const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& f)
{
	
	SetState(x);
	if(t==0)
		mSoftWorld->SetPositions(mSoftWorldX0);
	// 	auto cons = mRigidWorld->getConstraintSolver()->mManualConstraints;
	// 	for(int i =0;i<cons.size();i++)
	// 	{
	// 		Eigen::Isometry3d T = ((dart::constraint::WeldJointConstraint*)cons[i].get())->mRelativeTransform;
	// 		BodyNdoe* bn1 = ((dart::constraint::WeldJointConstraint*)cons[i].get())->mBodyNode1;
	// 		BodyNdoe* bn2 = ((dart::constraint::WeldJointConstraint*)cons[i].get())->mBodyNode2;

	// 		mRigidWorld->getConstraintSolver()->removeConstraint(cons[i]);

	// 		mRigidWorld->getConstraintSolver()->addConstraint(bn1,bn2);
	// 	}
	// }
	SetControl(u,t);
	Step();
	GetState(f);
}

void
MusculoSkeletalLQR::
SetState(const Eigen::VectorXd& x)
{
	mMusculoSkeletalSystem->GetSkeleton()->setPositions(x.head(mDofs));
	mMusculoSkeletalSystem->GetSkeleton()->setVelocities(x.block(mDofs,0,mDofs,1));
	
	mMusculoSkeletalSystem->GetSkeleton()->computeForwardKinematics(true,true,false);

	for(int i = 0;i<mBalls.size();i++)
	{
		mBalls[i]->GetSkeleton()->setPositions(x.block(2*mDofs+6*i+0,0,3,1));
		mBalls[i]->GetSkeleton()->setVelocities(x.block(2*mDofs+6*i+3,0,3,1));
		mBalls[i]->GetSkeleton()->computeForwardKinematics(true,true,false);	
	}
	
}
void
MusculoSkeletalLQR::
SetControl(const Eigen::VectorXd& u,double t)
{
	mTargetPositions = u+mReferenceMotions[t];
	if(t == 0){
		mTargetVelocities = (mReferenceMotions[t+1] - mReferenceMotions[t])/mSoftWorld->GetTimeStep();
	}
	else{
		mTargetVelocities = (mTargetPositions-(mReferenceMotions[t-1]+mu[t-1]))/mSoftWorld->GetTimeStep();
	}
}
void
MusculoSkeletalLQR::
GetState(Eigen::VectorXd& x)
{
	x.head(mDofs) = mMusculoSkeletalSystem->GetSkeleton()->getPositions();
	x.block(mDofs,0,mDofs,1) = mMusculoSkeletalSystem->GetSkeleton()->getVelocities();

	for(int i = 0;i<mBalls.size();i++)
	{
		x.block(2*mDofs+6*i+0,0,3,1) = mBalls[i]->GetSkeleton()->getPositions();
		x.block(2*mDofs+6*i+3,0,3,1) = mBalls[i]->GetSkeleton()->getVelocities();
	}

	
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
	// static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->Update(qdd_desired);
	// mMuscleOptimizationSolver->ReOptimizeTNLP(mMuscleOptimization);

	// Eigen::VectorXd solution =  static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->GetSolution();

	// mMusculoSkeletalSystem->SetActivationLevels(solution.tail(mMusculoSkeletalSystem->GetNumMuscles()));
	// mMusculoSkeletalSystem->TransformAttachmentPoints();
	// mSoftWorld->TimeStepping(false);

	double nn = mSoftWorld->GetTimeStep() / mRigidWorld->getTimeStep();
	if(is_ioing){
		// std::cout<<mTargetPositions.transpose()<<std::endl;
		// std::cout<<mTargetVelocities.transpose()<<std::endl;
		// std::cout<<qdd_desired.transpose()<<std::endl;
	}
	for(int i =0; i<nn;i++)
	{
		// mMusculoSkeletalSystem->ApplyForcesToSkeletons(mSoftWorld);
		// mMusculoSkeletalSystem->GetSkeleton()->clearConstraintImpulses();
		// mMusculoSkeletalSystem->GetSkeleton()->clearInternalForces();
		Eigen::VectorXd torque = 	mMusculoSkeletalSystem->GetSkeleton()->getMassMatrix()*qdd_desired+
									mMusculoSkeletalSystem->GetSkeleton()->getCoriolisAndGravityForces();
		if(is_ioing)
		{
			// auto interesting = mRigidWorld->getConstraintSolver()->mManualConstraints[0];
			// for(int i =0;i<6;i++)
			// std::cout<<((dart::constraint::WeldJointConstraint*)interesting.get())->mJacobian2<<std::endl;
			// dart::constraint::ConstraintInfo* ci = new dart::constraint::ConstraintInfo();
			// mRigidWorld->getConstraintSolver()->mManualConstraints[0]->getInformation(ci);
			// qdd_desired<<
			// std::cout<<(torque).transpose()<<std::endl;
			// std::cout<<(mMusculoSkeletalSystem->GetSkeleton()->getMassMatrix())<<std::endl<<std::endl;

			// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getExternalForces().transpose()<<std::endl;
			// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getConstraintForces().transpose()<<std::endl;
			// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getConstraintForces().transpose()<<std::endl;
			// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getPositions()[0]<<" ";
			// std::cout<<mBalls[mBallIndex]->GetVelocity().transpose()<<std::endl;
		}
		mMusculoSkeletalSystem->GetSkeleton()->setForces(torque);

		mRigidWorld->step();
	}
}


void
MusculoSkeletalLQR::
Finalize()
{
	// std::cout<<"X : "<<std::endl;
	// std::cout<<"Before : \n"<<std::endl;
	// for(int t =0;t<mN;t++)
	// {
	// 	std::cout<<mx[t].transpose()<<std::endl;
	// }
	is_ioing = true;
	for(int t = 0;t<mN-1;t++)
	{
		Evalf(mx[t],mu[t],t,mx[t+1]);
		// std::cout<<mx[t].block(0,0,mDofs,1).transpose()<<std::endl;
		// SetState(mx[t]);
		// SetControl(mu[t],t);
		// std::cout<<mTargetPositions.transpose()<<std::endl;
		// std::cout<<mTargetVelocities.transpose()<<std::endl;
		// std::cout<<mBalls[mBallIndex]->GetVelocity().transpose()<<std::endl;
	}
	// std::cout<<"After : \n"<<std::endl;
	// for(int t =0;t<mN;t++)
	// {
		// std::cout<<mx[t].transpose()<<std::endl;
	// }

	// for(int t = 0;t<mN-1;t++)
	// {
		// Evalf(mx[t],mu[t],t,mx[t+1]);
		// std::cout<<mx[t].block(0,0,mDofs,1).transpose()<<std::endl;
		// SetState(mx[t]);
		// SetControl(mu[t],t);
		// std::cout<<mTargetPositions.transpose()<<std::endl;
		// std::cout<<mTargetVelocities.transpose()<<std::endl;
		// std::cout<<mBall->GetVelocity().transpose()<<std::endl;
	// }
	// std::cout<<"After2 : \n"<<std::endl;
	// for(int t =0;t<mN;t++)
	// {
	// 	std::cout<<mx[t].transpose()<<std::endl;
	// }

	double cf;
	EvalCf(mx[mN-1],cf);
	std::cout<<cf<<std::endl<<std::endl;
	// std::cout<<std::endl;
	// std::cout<<"U : "<<std::endl;
	// for(int t = 0;t<mN-1;t++)
	// {
	// 	std::cout<<(mReferenceMotions[t]+mu[t]).transpose()<<std::endl;
	// }

	// for(int t=0;t<mN-1;t++)
	// {
		
		

		
		// std::cout<<skel->getPositions().transpose()<<std::endl;
		// std::cout<<pos_diff.transpose()<<std::endl;
		// std::cout<<(mTargetVelocities - skel->getVelocities()).transpose()<<std::endl;
		
		// std::cout<<mx[t].block(0,0,mDofs,1).transpose()<<std::endl;
		// std::cout<<mx[t].block(mDofs,0,mDofs,1).transpose()<<std::endl;
		// Evalf(mx[t],mu[t],t,mx[t+1]);
		// auto& skel = mMusculoSkeletalSystem->GetSkeleton();
		// Eigen::VectorXd pos_diff = skel->getPositionDifferences(mTargetPositions,skel->getPositions());
		// for(int i = 0;i<pos_diff.rows();i++)
		// 	pos_diff[i] = dart::math::wrapToPi(pos_diff[i]);
		// Eigen::VectorXd qdd_desired = pos_diff.cwiseProduct(mKp) + (mTargetVelocities - skel->getVelocities()).cwiseProduct(mKv);
		// std::cout<<mTargetPositions.transpose()<<std::endl;
		// std::cout<<mTargetVelocities.transpose()<<std::endl;
		// std::cout<<qdd_desired.transpose()<<std::endl;
		// std::cout<<std::endl;
	// }	

	// std::cout<<std::endl;

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
		if(i<2*mDofs)//||(i>=2*mDofs+6*mBallIndex&&i<2*mDofs+6*mBallIndex+6 ))
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
		else
			fx.col(i).setZero();

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
