#include "IntegratedWorld.h"
#include "DART_helper.h"
#include "MusculoSkeletalSystem.h"
#include "Controller.h"
#include "Record.h"
using namespace FEM;
using namespace dart::dynamics;
using namespace dart::simulation;
IntegratedWorld::
IntegratedWorld()
{

}
bool
IntegratedWorld::
TimeStepping()
{
	bool need_fem_update = false;

	// std::cout<<"timestepping"<<std::endl;
	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getPositions().transpose()<<std::endl;

	if(mSoftWorld->GetTime()<=mRigidWorld->getTime())
	{
		need_fem_update = true;
		// std::cout<<"mController step"<<std::endl;
		mController->Step();
	}

	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getPositions().transpose()<<std::endl;
	// std::cout<<std::endl;
	// auto pd_forces = mController->ComputePDForces();
	mMusculoSkeletalSystem->GetSkeleton()->clearConstraintImpulses();
	mMusculoSkeletalSystem->GetSkeleton()->clearInternalForces();

	Eigen::VectorXd pd_forces = mMusculoSkeletalSystem->GetSkeleton()->getMassMatrix()*mController->mPDForces + mMusculoSkeletalSystem->GetSkeleton()->getCoriolisAndGravityForces();
	// std::cout<<mBalls[0]->GetPosition().transpose()<<std::endl;
	// auto interesting = mRigidWorld->getConstraintSolver()->mManualConstraints[0];
	// std::cout<<((dart::constraint::WeldJointConstraint*)interesting.get())->mJacobian2<<std::endl;
	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getPositions()[0]<<" ";
	// std::cout<<mBalls[0]->GetVelocity().transpose()<<std::endl;
			// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getExternalForces().transpose()<<std::endl;
	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getConstraintForces().transpose()<<std::endl;
	// std::cout<<pd_forces.transpose()<<std::endl;
	mMusculoSkeletalSystem->GetSkeleton()->setForces(pd_forces);



	// mMusculoSkeletalSystem->ApplyForcesToSkeletons(mSoftWorld);
	if(need_fem_update)
	{
		// mMusculoSkeletalSystem->TransformAttachmentPoints();
		mSoftWorld->SetTime(mSoftWorld->GetTime()+mSoftWorld->GetTimeStep());
		// mSoftWorld->TimeStepping();
	}

	mRigidWorld->step();

	mRecords.push_back(Record::Create());
	auto rec = mRecords.back();

	rec->Set(mRigidWorld,mSoftWorld,mMusculoSkeletalSystem,mController);

	return need_fem_update;
}
std::shared_ptr<IntegratedWorld>
IntegratedWorld::
Clone()
{
	auto new_iw = Create();

	new_iw->mSoftWorld = mSoftWorld->Clone();
	new_iw->mRigidWorld = mRigidWorld->clone();
	new_iw->mMusculoSkeletalSystem = mMusculoSkeletalSystem->Clone(mSoftWorld,mRigidWorld);
	
	return new_iw;
}
std::shared_ptr<IntegratedWorld>
IntegratedWorld::
Create()
{
	auto iw = new IntegratedWorld();
	return std::shared_ptr<IntegratedWorld>(iw);
}

void
IntegratedWorld::
Initialize()
{
	mSoftWorld = FEM::World::Create(
		FEM::IntegrationMethod::PROJECTIVE_QUASI_STATIC,	//Integration Method
		// FEM::IntegrationMethod::QUASI_STATIC,	//Integration Method
		// FEM::IntegrationMethod::PROJECTIVE_DYNAMICS,	//Integration Method
		1.0/200.0,							//Time Step
		100,								//Max Iteration
		Eigen::Vector3d(0,-9.81,0),					//Gravity
		0.999								//Damping
		);
	mRigidWorld = std::make_shared<dart::simulation::World>();
	mRigidWorld->setGravity(Eigen::Vector3d(0,-9.81,0));
	mRigidWorld->checkCollision();
	// mRigidWorld->setGravity(Eigen::Vector3d(0,0,0));
	mMusculoSkeletalSystem = MusculoSkeletalSystem::Create();
	MakeSkeleton(mMusculoSkeletalSystem);
	MakeMuscles("../vmcon/export/muscle_params.xml",mMusculoSkeletalSystem);

	mMusculoSkeletalSystem->Initialize(mSoftWorld,mRigidWorld);
	mSoftWorld->Initialize();

	for(int i =0;i<5;i++)
	{
		SkeletonPtr skel = Skeleton::create("ball_"+std::to_string(i));

		bool is_left_hand = i%2;
		if(is_left_hand)
		{
			auto* abn =mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandL");
			Eigen::Vector3d loc = abn->getTransform().translation();
			MakeBall(skel,abn->getCOM(),0.036,0.13);

			// mBalls.push_back(std::make_shared<Ball>(std::make_shared<dart::constraint::WeldJointConstraint>(skel->getBodyNode(0),abn),skel));
			mBalls.push_back(std::make_shared<Ball>(nullptr,skel));
			mRigidWorld->addSkeleton(skel);
			mBalls.back()->Attach(mRigidWorld,abn);
		}
		else
		{
			auto* abn =mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR");
			Eigen::Vector3d loc = abn->getTransform().translation();
			MakeBall(skel,abn->getCOM(),0.036,0.13);

			// mBalls.push_back(std::make_shared<Ball>(std::make_shared<dart::constraint::WeldJointConstraint>(skel->getBodyNode(0),abn),skel));
			mBalls.push_back(std::make_shared<Ball>(nullptr,skel));
			mRigidWorld->addSkeleton(skel);
			mBalls.back()->Attach(mRigidWorld,abn);	
		}
	
	}

	// for(int i=0;i<1;i++)
	// {
		
	// 	BodyNode* abn = mMusculoSkeletalSystem->GetSkeleton()->getBodyNode((i%2==0?"HandL":"HandR"));
	// 	MakeBall(skel,abn->getCOM(),0.036,0.13);
	// 	mBalls.push_back(std::make_shared<Ball>(skel));
	// 	mBalls.back()->Attach(mRigidWorld,abn);
	// 	mRigidWorld->addSkeleton(skel);
	// }
	mController = Controller::Create(mSoftWorld,mRigidWorld,mMusculoSkeletalSystem,mBalls);
}

void
IntegratedWorld::
SetRecord(int& frame)
{
	if(frame < 0)
		frame = 0;
	if(frame>=mRecords.size())
		frame=0;
	mRecords[frame]->Get(mRigidWorld,mSoftWorld,mMusculoSkeletalSystem,mController);

}