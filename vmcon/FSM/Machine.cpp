#include "Machine.h"
#include "../MusculoSkeletalSystem.h"
#include "../MuscleOptimization.h"
#include "../IKOptimization.h"
#include "../Ball.h"
#include "../iLQR/MusculoSkeletalLQR.h"
#include "../DART_helper.h"
#include "BezierCurve.h"
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Ipopt;
Machine::
Machine(const dart::simulation::WorldPtr& rigid_world,
		const FEM::WorldPtr& soft_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls,
	const std::vector<int>& sequences,int ball_size)
	:mRigidWorld(rigid_world),mSoftWorld(soft_world),mMusculoSkeletalSystem(musculo_skeletal_system),mBalls(balls),
	mJugglingInfo(new JugglingInfo(sequences,ball_size)),mLocalOffset(Eigen::Vector3d(0.0,0.02,0.03))
{
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

	mHandX0 = l_loc;
	mIKSolver->OptimizeTNLP(mIKOptimization);
	mPhase = 0;
	InitializeLQR();
	mCurve = new BezierCurve();


}

void
Machine::
GetMotion(Eigen::VectorXd& p,Eigen::VectorXd& v)
{
	auto& skel = mMusculoSkeletalSystem->GetSkeleton();
	auto ball = mBalls[mJugglingInfo->GetBallIndex()];
	auto bn_from = skel->getBodyNode(mJugglingInfo->From());
	auto bn_to = skel->getBodyNode(mJugglingInfo->To());
	//Check need to update.
	bool need_update = false;
	if(mJugglingInfo->GetCount()==1){
		exit(0);
	}
	//Check Catch Phase Finished.
	if(mPhase == 0)
	{
		Eigen::Vector3d body_position = bn_from->getTransform()*mLocalOffset;
		Eigen::Vector3d ball_position = ball->GetPosition();
		
		if(ball->IsReleased())
			GenerateCatchMotions();

		if((body_position-ball_position).norm()<5E-2 || !ball->IsReleased())
		{
			ball->Attach(mRigidWorld,bn_from);
			need_update = true;
			mPhase = 1;
		}
	}
	//Look opposite Hand
	mJugglingInfo->CountPlusPlus();
	ball = mBalls[mJugglingInfo->GetBallIndex()];
	bn_from = skel->getBodyNode(mJugglingInfo->From());

	if(ball->IsReleased()) // check if already attached.
	{
		Eigen::Vector3d body_position = bn_from->getTransform()*mLocalOffset;

		Eigen::Vector3d ball_position = ball->GetPosition();
		if((body_position-ball_position).norm()<5E-2){
			ball->Attach(mRigidWorld,bn_from);
		}
	}
	//Go back
	mJugglingInfo->CountMinusMinus();
	ball = mBalls[mJugglingInfo->GetBallIndex()];
	bn_from = skel->getBodyNode(mJugglingInfo->From());
	//Check Swing Phase Finished.
	if(mPhase ==1 &&!need_update)
	{
		// std::cout<<ball->GetVelocity().transpose()<<std::endl;
		if(mCount == mU.size() || mMotions.back().second<mTimeElapsed)
		{
			if(mJugglingInfo->GetV()!=0){
			ball->Release(mRigidWorld);
			std::cout<<"Released Velocity : "<<mBalls[mJugglingInfo->GetBallIndex()]->releasedVelocity.transpose()<<std::endl;
			}
			//Look Ahead for targeting next ball. (count+2 -> next target)
			mJugglingInfo->CountPlusPlus();
			if(mJugglingInfo->GetV()!=1)
			{
				mJugglingInfo->CountPlusPlus();
				if(mBalls[mJugglingInfo->GetBallIndex()]->IsReleased())
				{
					// std::cout<<"Look ahead"<<mJugglingInfo->GetBallIndex()<<std::endl;
					GenerateCatchMotions();
				}
				mJugglingInfo->CountMinusMinus();
			}
			
			//to next state.
			need_update = true;
			mPhase = 0;
		}
	}

	if(need_update)
	{
		if(mPhase == 0)
			GenerateCatchMotions();
		else
			GenerateSwingMotions();
	}

	p = mMotions[mCount].first;
	if(mCount == 0)
		v = (mMotions[mCount+1].first - mMotions[mCount].first)/mSoftWorld->GetTimeStep();
	else
		v = (p-mMotions[mCount-1].first)/mSoftWorld->GetTimeStep();
	mTimeElapsed += mSoftWorld->GetTimeStep();
	mCount++;
}

void
Machine::
GenerateCatchMotions()
{
	auto& skel = mMusculoSkeletalSystem->GetSkeleton();
	auto& ball = mBalls[mJugglingInfo->GetBallIndex()];
	auto bn_from = skel->getBodyNode(mJugglingInfo->From());
	auto bn_to = skel->getBodyNode(mJugglingInfo->To());

	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));

	//Check if IK target update is needed.
	Eigen::Vector3d target;
	// ball->ComputeFallingPosition(ball->releasedPoint[1],target);
	ball->ComputeFallingPosition(mHandX0[1],target);
	if(target.norm()<1E-6)
		return;
	// std::cout<<"Catch Target : "<<target.transpose()<<std::endl;
	AnchorPoint ap = std::make_pair(bn_from,mLocalOffset);
	ik->AddTargetPositions(ap,target);
	mIKSolver->ReOptimizeTNLP(mIKOptimization);	

	mMotions.clear();
	//Add motions
	for(int i =0;i<1000;i++)
		mMotions.push_back(std::make_pair(ik->GetSolution(),100.0));
	mCount = 0;
	mTimeElapsed = 0.0;
}
void
Machine::
GenerateSwingMotions()
{
	//Initial Guess
	auto& skel = mMusculoSkeletalSystem->GetSkeleton();
	auto& ball = mBalls[mJugglingInfo->GetBallIndex()];
	auto bn_from = skel->getBodyNode(mJugglingInfo->From());
	auto bn_to = skel->getBodyNode(mJugglingInfo->To());
	double t_hold = mJugglingInfo->GetT_hold();
	double t_free = mJugglingInfo->GetT_free();


	Eigen::Vector3d dir = bn_to->getCOM() - ball->GetPosition();	
	Eigen::Vector3d p0,p1,p2,v2;

	//Make Bezier curve
	p0 = ball->GetPosition();
	
	// Singular case V = 0
	if(mJugglingInfo->GetV() == 0)
	{
		mTimeElapsed = 0.0;
		mCount = 0;
		return;
	}
	//Singular case V = 1
	if(mJugglingInfo->GetV() == 1)
	{
		p0[1] = 0.2;
		p2 = p0 + 0.2*dir;

		Eigen::Vector3d diff = bn_to->getTransform()*mLocalOffset - p0;
		double t_hold = mJugglingInfo->GetT()*0.5;

		v2[1] = 0.5*9.81*t_free;
		v2[0] = diff[0]/t_free;
		v2[2] = diff[2]/t_free;
		
		p1 = p2 - 0.5*v2*t_hold;

		std::cout<<p0.transpose()<<std::endl;
		std::cout<<p1.transpose()<<std::endl;
		std::cout<<p2.transpose()<<std::endl;
		mCurve->Initialize(p0,p1,p2,t_hold);
	}
	else
	{
		p0 = bn_from->getTransform().translation();
		p0[0] *= 1.2;
		p2 = mHandX0;
		p2[0] *= 0.8;
		p2[1] += 0.1;
		if(ball->GetPosition()[0]<0){
			p2[0] =-p2[0];
		}
		
		// if(std::abs(p0[0])<std::abs(mHandX0[0]))
			// p2[0] = p0[0]*1.1;
		// else	
			// p2[0] = p0[0]*0.9;
		
		
		// p0[0] *=1.1;
		// p2[0] *=0.9;
		Eigen::Vector3d target_to = bn_to->getTransform()*mLocalOffset;
		if(bn_from == bn_to)
			target_to = p2;
		else
		{
			target_to = p2;
			target_to[0] = -target_to[0];
		}
		target_to[1] -=0.1;
		v2 = mJugglingInfo->GetTargetVelocity(p2,target_to);
		std::cout<<v2.transpose()<<std::endl;
		p1 = p2 - 0.5*v2*t_hold;
		std::cout<<p0.transpose()<<std::endl;
		std::cout<<p1.transpose()<<std::endl;
		std::cout<<p2.transpose()<<std::endl;
		mCurve->Initialize(p0,p1,p2,t_hold);
	}
	//Solve IK to make coarse_motions
	std::vector<std::pair<Eigen::VectorXd,double>> coarse_motions;

	IKOptimization* ik = static_cast<IKOptimization*>(GetRawPtr(mIKOptimization));
	Eigen::VectorXd save_positions = ik->GetSolution();

	auto save_target = ik->GetTargets();
	Eigen::Vector3d p_hb = ball->GetPosition() - bn_from->getCOM();
	AnchorPoint ap = std::make_pair(bn_from,p_hb);
	for(int i =0;i<11;i++)
	{
		double tt = ((double)i)/((double)10)*t_hold;
		
		Eigen::Vector3d p_ee = mCurve->GetPosition(tt);
		// std::cout<<"p_ee("<<i<<")"<<p_ee.transpose()<<std::endl;
		ik->AddTargetPositions(ap,p_ee);
		mIKSolver->ReOptimizeTNLP(mIKOptimization);
		Eigen::VectorXd sol = ik->GetSolution();
		coarse_motions.push_back(std::make_pair(sol,tt));
	}

	for(auto& target : save_target){
		ik->AddTargetPositions(target.first,target.second);
	}

	ik->SetSolution(save_positions);

	mMotions = GenerateFineMotions(coarse_motions);

	//Solve LQR
	SynchronizeLQR();
	OptimizeLQR(p2,v2);
	
	mTimeElapsed = 0.0;
	mCount = 0;
}

std::vector<std::pair<Eigen::VectorXd,double>>
Machine::
GenerateFineMotions(const std::vector<std::pair<Eigen::VectorXd,double>>& coarse_motions)
{
	std::vector<std::pair<Eigen::VectorXd,double>> motions;
	//Make fine_motions by interpolation

	Eigen::VectorXd p,v;
	double time_elapsed=0.0;

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
		double t1 = t+0.001;
		auto pdp =(1.0-t1/dt)*(coarse_motions[k].first) + (t1/dt)*(coarse_motions[k1].first);

		v = (pdp-p)/0.001;
		motions.push_back(std::make_pair(p,time_elapsed));
		time_elapsed+=mSoftWorld->GetTimeStep();
	}
	return motions;
}
void
Machine::
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

	for(int i =0;i<mBalls.size();i++)
	{
		SkeletonPtr skel = Skeleton::create("ball_"+std::to_string(i));

		bool is_left_hand = i%2;
		if(is_left_hand)
		{
			auto* abn =mLQRMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandL");
			Eigen::Vector3d loc = abn->getTransform().translation();
			loc += mLocalOffset;
			MakeBall(skel,mBalls[i]->GetPosition(),0.036,mBalls[i]->GetSkeleton()->getBodyNode(0)->getMass());

			mLQRBalls.push_back(std::make_shared<Ball>(nullptr,skel));
			mLQRRigidWorld->addSkeleton(skel);
			mLQRBalls.back()->Attach(mLQRRigidWorld,abn);
		}
		else
		{
			auto* abn =mLQRMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR");
			Eigen::Vector3d loc = abn->getTransform().translation();
			loc += mLocalOffset;
			MakeBall(skel,mBalls[i]->GetPosition(),0.036,mBalls[i]->GetSkeleton()->getBodyNode(0)->getMass());

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
void
Machine::
SynchronizeLQR()
{
	mLQRSoftWorld->SetPositions(mSoftWorld->GetPositions());

	for(int i =0;i<mLQRRigidWorld->getNumSkeletons();i++){
		mLQRRigidWorld->getSkeleton(i)->setPositions(mRigidWorld->getSkeleton(i)->getPositions());
		mLQRRigidWorld->getSkeleton(i)->setVelocities(mRigidWorld->getSkeleton(i)->getVelocities());
		mLQRRigidWorld->getSkeleton(i)->computeForwardKinematics();
	}
	mLQRMusculoSkeletalSystem->TransformAttachmentPoints();
	mLQRMusculoSkeletalSystem->SetActivationLevels(mMusculoSkeletalSystem->GetActivationLevels());
	// for(int i=0;i<mBalls.size();i++)
	// {
	// 	std::cout<<i<<"  : "<<mBalls[i]->GetPosition().transpose()<<std::endl;
	// 	std::cout<<i<<"  : "<<mLQRBalls[i]->GetPosition().transpose()<<std::endl;
	// }


	for(int i =0;i<mBalls.size();i++)
	{
		if(!mBalls[i]->IsReleased())
		{
			mLQRBalls[i]->Release(mLQRRigidWorld);
			mLQRBalls[i]->Attach(
				mLQRRigidWorld,
				mLQRMusculoSkeletalSystem->GetSkeleton()->getBodyNode(mBalls[i]->GetConstraint()->getBodyNode2()->getName()));
		}
		else
		{
			mLQRBalls[i]->Release(mLQRRigidWorld);
		}
	}

	for(int i=0;i<mLQRRigidWorld->getNumSkeletons();i++)
	{
		mLQRRigidWorld->getSkeleton(i)->clearConstraintImpulses();
		mLQRRigidWorld->getSkeleton(i)->clearInternalForces();
		mRigidWorld->getSkeleton(i)->clearConstraintImpulses();
		mRigidWorld->getSkeleton(i)->clearInternalForces();
	}
}
void
Machine::
OptimizeLQR(const Eigen::Vector3d& p_des,const Eigen::Vector3d& v_des)
{
	int dofs =mLQRMusculoSkeletalSystem->GetSkeleton()->getNumDofs();
	Eigen::VectorXd x0(dofs*2+12*mBalls.size()+mSoftWorld->GetPositions().rows());
	x0.head(dofs) = mLQRMusculoSkeletalSystem->GetSkeleton()->getPositions();
	x0.block(dofs,0,dofs,1) = mLQRMusculoSkeletalSystem->GetSkeleton()->getVelocities();
	for(int i =0;i<mBalls.size();i++)
	{
		x0.block(2*dofs+12*i,0,6,1) = mLQRBalls[i]->GetSkeleton()->getPositions();
		x0.block(2*dofs+12*i+6,0,6,1) = mLQRBalls[i]->GetSkeleton()->getVelocities();
	}
	x0.tail(mSoftWorld->GetPositions().rows()) = mSoftWorld->GetPositions();
	std::vector<Eigen::VectorXd> ref,u0;
	ref.resize(mMotions.size());
	u0.resize(mMotions.size()-1);

	for(int i=0;i<u0.size();i++){
		ref[i] = mMotions[i].first;
		u0[i] = ref[i];
		u0[i].setZero();
	}
	ref.back() = mMotions.back().first;
	mJugglingInfo->CountPlusPlus();
	int next_index = mJugglingInfo->GetBallIndex();
	BodyNode* next_body =  mLQRMusculoSkeletalSystem->GetSkeleton()->getBodyNode(mJugglingInfo->From());
	bool next_ball_initially_attached = !mBalls[mJugglingInfo->GetBallIndex()]->IsReleased();
	mJugglingInfo->CountMinusMinus();
	mLQR->Initialze(p_des,v_des,mJugglingInfo->GetBallIndex(),next_index,next_body,next_ball_initially_attached,ref,x0,u0);
	mU = mLQR->Solve();
	// mU = u0;

	for(int i=0;i<mU.size();i++)
	{
		mMotions[i].first +=mU[i];
		// std::cout<<mU[i].transpose()<<std::endl;
	}

	mMotions.back().first +=mU.back();

}