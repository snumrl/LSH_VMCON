#include "../MusculoSkeletalSystem.h"
#include "../MuscleOptimization.h"
#include "../IKOptimization.h"
#include "../Ball.h"
#include "../iLQR/MusculoSkeletalLQR.h"
#include "../DART_helper.h"
#include "BezierCurve.h"
#include "Sampler.h"
#include <boost/filesystem.hpp>
#include <chrono>

#include <tinyxml.h>

#include <fstream>
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Ipopt;


Sampler::
Sampler(
		const dart::simulation::WorldPtr& rigid_world,
		const std::shared_ptr<FEM::World>& soft_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls)
		:mRigidWorld(rigid_world),mSoftWorld(soft_world),mMusculoSkeletalSystem(musculo_skeletal_system),mBalls(balls),
		mKp(Eigen::VectorXd::Constant(musculo_skeletal_system->GetSkeleton()->getNumDofs(),800.0)),
		mKv(Eigen::VectorXd::Constant(musculo_skeletal_system->GetSkeleton()->getNumDofs(),2*sqrt(800.0)))
{
	int dofs = musculo_skeletal_system->GetSkeleton()->getNumDofs();
	for(int i =0;i<6;i++)
	{
		mKp[dofs-1-i] = 2.0*mKp[dofs-1-i];	
		mKv[dofs-1-i] = sqrt(2.0)*mKv[dofs-1-i];
	}
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

	mIKOptimization = new IKOptimization(mMusculoSkeletalSystem->GetSkeleton());

	mIKSolver = new IpoptApplication();
	mIKSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	mIKSolver->Options()->SetStringValue("jac_c_constant", "yes");
	mIKSolver->Options()->SetStringValue("hessian_constant", "yes");
	mIKSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	mIKSolver->Options()->SetIntegerValue("print_level", 2);
	mIKSolver->Options()->SetIntegerValue("max_iter", 1000);
	mIKSolver->Options()->SetNumericValue("tol", 1e-4);


	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));

	Eigen::Vector3d l_loc = mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandL")->getCOM();
	Eigen::Vector3d r_loc = mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR")->getCOM();

	ik->AddTargetPositions(std::make_pair(mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandL"),Eigen::Vector3d::Zero()),l_loc);
	ik->AddTargetPositions(std::make_pair(mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR"),Eigen::Vector3d::Zero()),r_loc);
	
	std::ifstream param("../vmcon/export/param2.txt");
	param>>w_regularization>>w_smooth>>w_pos_track>>w_vel_track;
	param.close();
	mInitCount = 0;
}

void
Sampler::
Initialze(
	const Eigen::Vector3d& pos_desired,
	const Eigen::Vector3d& vel_desired,
	int index,BodyNode* body,const std::vector<std::pair<AnchorPoint,Eigen::Vector3d>>& ik_targets,
	int next_index,BodyNode* next_body,bool next_ball_initially_attached,const Eigen::VectorXd& s0,double t_hold)
{
	mTHold = t_hold;
	mS0 = s0;
	mBallIndex = index;
	mBody = body;
	mNextBallIndex = next_index;
	mNextBody = next_body;
	mNextBallInitiallyAttached = next_ball_initially_attached;
	std::cout<<"p_t : "<<pos_desired.transpose()<<std::endl;
	std::cout<<"v_t : "<<vel_desired.transpose()<<std::endl;
	mBallTargetPosition = pos_desired;
	mBallTargetVelocity = vel_desired;

	mWritePath = "../output_curve"+std::to_string((int)mInitCount++);
	boost::filesystem::create_directories(mWritePath);
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
	ik->ClearTarget();
	for(auto& target : ik_targets){
		// std::cout<<target.first.first->getName()<<target.second<<std::endl;
		ik->AddTargetPositions(std::make_pair(mMusculoSkeletalSystem->GetSkeleton()->getBodyNode(target.first.first->getName()),target.first.second),target.second);
	}
}

Eigen::VectorXd
Sampler::
Solve(const Eigen::VectorXd& x0)
{
	Eigen::VectorXd x_star = x0;
	std::ofstream ofs("out"+std::to_string(mInitCount));
	ofs<<x_star.transpose()<<std::endl;
	Finalize(x_star);
	for(int i =0;i<15;i++)
	{
		double c = EvalC(x_star);
		Eigen::Vector3d ball_vel = mBalls[mBallIndex]->GetVelocity();
		ofs<<x_star.transpose()<<std::endl;
		auto start = std::chrono::system_clock::now();
		Eigen::VectorXd cx = EvalCx(x_star);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end-start;
	    std::cout<<"EvalCx : " << elapsed_seconds.count() << "s"<<std::endl;

		// Eigen::MatrixXd cxx = EvalCxx(x_star);
		// Eigen::MatrixXd cxx_inv = cxx.inverse();
		Eigen::VectorXd dir = -cx;
		double alpha = 0.2;
		Eigen::VectorXd x_next;
		
		bool is_updated = false;
		start = std::chrono::system_clock::now();
		for(int j=0;j<10;j++)
		{
			x_next = x_star + alpha*dir;
			double c_next = EvalC(x_next);
			std::cout<<"a : "<<alpha<<" :"<<c_next<<std::endl;
			if(c_next<c)
			{
				c = c_next;
				is_updated =true;
				break;
			}
			alpha*=0.5;
		}
		end = std::chrono::system_clock::now();
		elapsed_seconds = end-start;
	    std::cout<<"Line Search : " << elapsed_seconds.count() << "s"<<std::endl;
		if(is_updated)
			x_star = x_next;
		ball_vel = mBalls[mBallIndex]->GetVelocity();
		std::cout<<ball_vel.transpose()<<std::endl;
		Finalize(x_star,std::to_string(i));


		if(!is_updated || c<1E-4)
			break;
	}
	
	ofs.close();
	return x_star;
}

double
Sampler::
EvalC(const Eigen::VectorXd& x)
{
	std::vector<Eigen::VectorXd> motions;
	GenerateMotions(x,motions);

	Simulate(motions);

	Eigen::Vector3d ball_pos = mBalls[mBallIndex]->GetPosition();
	Eigen::Vector3d ball_vel = mBalls[mBallIndex]->GetVelocity();
	// std::cout<<"V : "<<ball_vel.transpose()<<std::endl;
	double cf;
	cf = 0.5*w_pos_track*(mBallTargetPosition - ball_pos).squaredNorm();
	cf += 0.5*w_vel_track*(mBallTargetVelocity - ball_vel).squaredNorm();
	return cf;
}

Eigen::VectorXd
Sampler::
EvalCx(const Eigen::VectorXd& x)
{
	Eigen::VectorXd cx(x.rows());
	cx.setZero();

	Eigen::VectorXd x_i;
	double delta = 0.01;
	double c_minus,c_plus;
	double x_i_plus,x_i_minus;
	for(int i = 0;i<x.rows();i++)
	{
		if(i>2)
		{
			x_i = x;
			c_minus =0;
			c_plus =0;
			x_i_minus = x[i] - delta;
			x_i_plus = x[i] + delta;
			
			x_i[i] = x_i_minus;
			c_minus = EvalC(x_i);
			x_i[i] = x_i_plus;
			c_plus = EvalC(x_i);

			cx[i] = (c_plus - c_minus)/(x_i_plus - x_i_minus);
		}
		else
		{
			cx[i] = 0.0;
		}
	}
	cx.block<3,1>(3,0) *= 2.0;
	// std::cout<<cx.transpose()<<std::endl;
	return cx;
}
Eigen::MatrixXd
Sampler::
EvalCxx(const Eigen::VectorXd& x)
{
	Eigen::MatrixXd cxx(x.rows(),x.rows());

	cxx.setZero();
	Eigen::VectorXd x_i;
	double delta = 0.01;
	Eigen::VectorXd cx_minus(x.rows()),cx_plus(x.rows());
	double x_i_minus,x_i_plus;

	for(int i = 0;i<x.rows();i++)
	{
		if(i>2)
		{
			x_i = x;
			cx_minus.setZero();
			cx_plus.setZero();
			x_i_minus = x[i] - delta;
			x_i_plus = x[i] + delta;

			x_i[i] = x_i_minus;
			cx_minus = EvalCx(x_i);
			x_i[i] = x_i_plus;
			cx_plus = EvalCx(x_i);

			cxx.col(i) = (cx_plus - cx_minus)/(x_i_plus - x_i_minus);
		}
		else
		{
			cxx.col(i).setZero();
			cxx(i,i) = 1.0;
		}

	}
	return cxx;
}
void
Sampler::
GenerateMotions(const Eigen::VectorXd& x,std::vector<Eigen::VectorXd>& motions)
{
	auto& skel = mMusculoSkeletalSystem->GetSkeleton();
	auto& ball = mBalls[mBallIndex];

	BezierCurve bc;
	// bc.Initialize(x.block<3,1>(0,0),x.block<3,1>(3,0),x.block<3,1>(6,0),1);
	bc.Initialize(x.block<3,1>(0,0),x.block<3,1>(3,0),x.block<3,1>(6,0),1);

	std::vector<std::pair<Eigen::VectorXd,double>> coarse_motions;

	SetS0();
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
	mIKSolver->Initialize();
	mIKSolver->OptimizeTNLP(mIKOptimization);
	auto save_target = ik->GetTargets();
	Eigen::Vector3d p_hb = ball->GetPosition() - mBody->getCOM();

	AnchorPoint ap = std::make_pair(mBody,p_hb);
	for(int i =0;i<11;i++)
	{
		double tt = ((double)i)/((double)10);
		
		Eigen::Vector3d p_ee = bc.GetPosition(tt);
		ik->AddTargetPositions(ap,p_ee);
		// std::cout<<p_ee.transpose()<<std::endl;
		mIKSolver->ReOptimizeTNLP(mIKOptimization);
		Eigen::VectorXd sol = ik->GetSolution();
		skel->setPositions(sol);
		skel->computeForwardKinematics(true,false,false);
		coarse_motions.push_back(std::make_pair(sol,tt*mTHold));
	}
	// std::cout<<std::endl;

	for(auto& target : save_target){
		// std::cout<<target.first.first->getName()<<target.second<<std::endl;
		ik->AddTargetPositions(target.first,target.second);
	}


	Eigen::VectorXd p;
	double time_elapsed = 0;
	while(true)
	{
		int k =0,k1 =0;
		for(int i =0;i<coarse_motions.size();i++)
		{
			if(coarse_motions[i].second<time_elapsed)
				k=i;
		}
		if(k ==coarse_motions.size()-1)
			break;
		k1 = k+1;
		double t = time_elapsed-coarse_motions[k].second;
		double dt = coarse_motions[k1].second-coarse_motions[k].second;

		p = (1.0-t/dt)*(coarse_motions[k].first) + (t/dt)*(coarse_motions[k1].first);

		motions.push_back(p);
		time_elapsed+=mSoftWorld->GetTimeStep();
	}
	SetS0();
}

void
Sampler::
Simulate(const std::vector<Eigen::VectorXd>& motions)
{
	SetS0();
	Eigen::VectorXd target_position,target_velocity;

	for(int t = 0;t<motions.size();t++)
	{
		target_position = motions[t];
		if(t==0)
			target_velocity = (motions[t+1]-motions[t])/mSoftWorld->GetTimeStep();
		else
			target_velocity = (motions[t]-motions[t-1])/mSoftWorld->GetTimeStep();
		Step(target_position,target_velocity);
	}	
}
void
Sampler::
SetS0()
{
	auto& skel = mMusculoSkeletalSystem->GetSkeleton();
	int dofs = skel->getNumDofs();
	
	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
	ik->SetSolution(mS0.head(skel->getNumDofs()));
	
	mMusculoSkeletalSystem->GetSkeleton()->setPositions(mS0.head(dofs));

	mMusculoSkeletalSystem->GetSkeleton()->setVelocities(mS0.block(dofs,0,dofs,1));

	mMusculoSkeletalSystem->GetSkeleton()->computeForwardKinematics(true,false,false);
	for(int i = 0;i<mBalls.size();i++)
	{
		mBalls[i]->GetSkeleton()->setPositions(mS0.block(2*dofs+12*i+0,0,6,1));
		mBalls[i]->GetSkeleton()->setVelocities(mS0.block(2*dofs+12*i+6,0,6,1));
		mBalls[i]->GetSkeleton()->computeForwardKinematics(true,false,false);	
	}

#ifndef USE_JOINT_TORQUE	
	mMusculoSkeletalSystem->SetActivationLevels(mS0.block(dofs*2+12*mBalls.size(),0,mMusculoSkeletalSystem->GetNumMuscles(),1));
	mSoftWorld->SetPositions(mS0.tail(mSoftWorld->GetPositions().rows()));
	mMusculoSkeletalSystem->TransformAttachmentPoints();
#endif
	mBalls[mBallIndex]->Attach(mRigidWorld,mBody);	
	if(mNextBallInitiallyAttached)
		mBalls[mNextBallIndex]->Attach(mRigidWorld,mNextBody);	
	else
		mBalls[mNextBallIndex]->Release(mRigidWorld);

}
void
Sampler::
Step(const Eigen::VectorXd& q,const Eigen::VectorXd& qd)
{
	auto& skel = mMusculoSkeletalSystem->GetSkeleton();
	Eigen::VectorXd pos_diff = skel->getPositionDifferences(q,skel->getPositions());
	for(int i = 0;i<pos_diff.rows();i++)
		pos_diff[i] = dart::math::wrapToPi(pos_diff[i]);
	Eigen::VectorXd qdd_desired = pos_diff.cwiseProduct(mKp) + (qd - skel->getVelocities()).cwiseProduct(mKv);
	if(mBalls[mNextBallIndex]->IsReleased()) // check if already attached.
	{
		Eigen::Vector3d body_position = mNextBody->getTransform()*Eigen::Vector3d(0.0,0.02,0.03);
		Eigen::Vector3d ball_position = mBalls[mNextBallIndex]->GetPosition();
		if((body_position-ball_position).norm()<5E-2){
			mBalls[mNextBallIndex]->Attach(mRigidWorld,mNextBody);
		}
	}
#ifndef USE_JOINT_TORQUE
	mMusculoSkeletalSystem->TransformAttachmentPoints();
	mSoftWorld->TimeStepping(false);
	static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->Update(qdd_desired);
	mMuscleOptimizationSolver->ReOptimizeTNLP(mMuscleOptimization);
	Eigen::VectorXd solution =  static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->GetSolution();
	mMusculoSkeletalSystem->SetActivationLevels(solution.tail(mMusculoSkeletalSystem->GetNumMuscles()));
	mSoftWorld->TimeStepping(false);
	double nn = mSoftWorld->GetTimeStep() / mRigidWorld->getTimeStep();
	for(int i =0; i<nn;i++)
	{
		mMusculoSkeletalSystem->ApplyForcesToSkeletons(mSoftWorld);
		mRigidWorld->step();
	}
#else
	double nn = mSoftWorld->GetTimeStep() / mRigidWorld->getTimeStep();

	for(int i =0; i<nn;i++)
	{
		Eigen::VectorXd torque = 	mMusculoSkeletalSystem->GetSkeleton()->getMassMatrix()*qdd_desired+
									mMusculoSkeletalSystem->GetSkeleton()->getCoriolisAndGravityForces();
		mMusculoSkeletalSystem->GetSkeleton()->setForces(torque);
		mRigidWorld->step();
	}
#endif
}


void
Sampler::
Finalize(const Eigen::VectorXd& x_star,const std::string& post_index)
{
	mWriteCount = 0;

	std::string path = mWritePath+"/iteration"+post_index;
	boost::filesystem::create_directories(path);
	mRigidWorld->setTime(0.0);
	SetS0();
	std::vector<Eigen::VectorXd> motions;
	GenerateMotions(x_star,motions);
	Eigen::VectorXd target_position,target_velocity;
	mTargetPositions = motions[0];
	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getPositions().transpose()<<std::endl;
	WriteRecord(path);
	for(int t = 0;t<motions.size();t++)
	{
		target_position = motions[t];
		if(t==0)
			target_velocity = (motions[t+1]-motions[t])/mSoftWorld->GetTimeStep();
		else
			target_velocity = (motions[t]-motions[t-1])/mSoftWorld->GetTimeStep();
		mTargetPositions = target_position;
		Step(target_position,target_velocity);
		WriteRecord(path);
	}	
	auto abn =mBalls[mBallIndex]->GetConstraint()->getBodyNode2();
	mBalls[mBallIndex]->Release(mRigidWorld);
	while(true)
	{
		if(mRigidWorld->getTime()>0.4)
			break;
		if(mBalls[mNextBallIndex]->IsReleased()) // check if already attached.
		{
			Eigen::Vector3d body_position = mNextBody->getTransform()*Eigen::Vector3d(0.0,0.02,0.03);
			Eigen::Vector3d ball_position = mBalls[mNextBallIndex]->GetPosition();
			if((body_position-ball_position).norm()<5E-2)
			{
				mBalls[mNextBallIndex]->Attach(mRigidWorld,mNextBody);
			}
		}
		target_position = motions.back();
		target_velocity.setZero();
		Step(target_position,target_velocity);
		mTargetPositions = target_position;
		WriteRecord(path);
	}
	mBalls[mNextBallIndex]->Release(mRigidWorld);
	mBalls[mBallIndex]->Release(mRigidWorld);
	SetS0();
}

void
Sampler::
WriteRecord(const std::string& path)
{
	std::string real_path = path +"/"+std::to_string(mWriteCount);
	std::ofstream ofs(real_path);
	Eigen::VectorXd soft_pos = mSoftWorld->GetPositions();
	ofs<<"soft "<<soft_pos.transpose()<<std::endl;

	for(int i =0;i<mRigidWorld->getNumSkeletons();i++)
	{
		ofs<<"rpos ";
		ofs<<mRigidWorld->getSkeleton(i)->getPositions().transpose()<<std::endl;
	}

	for(int i =0;i<mRigidWorld->getNumSkeletons();i++)
	{
		ofs<<"rvel ";
		ofs<<mRigidWorld->getSkeleton(i)->getVelocities().transpose()<<std::endl;
	}
	ofs<<"target "<<mTargetPositions.transpose()<<std::endl;
	ofs<<"time "<<mRigidWorld->getTime()<<std::endl;
	ofs<<"act "<<mMusculoSkeletalSystem->GetActivationLevels().transpose()<<std::endl;
	ofs.close();
	mWriteCount++;
}


