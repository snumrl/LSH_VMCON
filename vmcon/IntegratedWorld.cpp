#include "IntegratedWorld.h"
#include "MusculoSkeletalSystem.h"
#include "Controller.h"
using namespace FEM;
using namespace dart::dynamics;
using namespace dart::simulation;
IntegratedWorld::
IntegratedWorld()
{

}
static int bbasdf=0;
bool
IntegratedWorld::
TimeStepping()
{
	bool need_fem_update = false;
	if(bbasdf++%33==0)
		need_fem_update=true;

	// if(mSoftWorld->GetTime()<mRigidWorld->getTime())
		// need_fem_update = true;

	// mMusculoSkeletalSystem->ApplyForcesToSkeletons(mSoftWorld);
	mMusculoSkeletalSystem->GetSkeleton()->setForces(
		mMusculoSkeletalSystem->GetSkeleton()->getMassMatrix()*mController->ComputePDForces() +
		mMusculoSkeletalSystem->GetSkeleton()->getCoriolisAndGravityForces()
		);
	// if(need_fem_update)
	// {
	// 	mMusculoSkeletalSystem->TransformAttachmentPoints();
	// 	mSoftWorld->TimeStepping();
	// }

	mRigidWorld->step();

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
		// FEM::IntegrationMethod::PROJECTIVE_DYNAMICS,	//Integration Method
		1.0/120.0,							//Time Step
		100,								//Max Iteration
		Eigen::Vector3d(0,-9.81,0),					//Gravity
		0.999								//Damping
		);
	mRigidWorld = std::make_shared<dart::simulation::World>();
	mRigidWorld->setGravity(Eigen::Vector3d(0,0,0));
	mMusculoSkeletalSystem = MusculoSkeletalSystem::Create();
	MakeSkeleton(mMusculoSkeletalSystem);
	MakeMuscles("../vmcon/export/muscle_params.xml",mMusculoSkeletalSystem);

	mMusculoSkeletalSystem->Initialize(mSoftWorld,mRigidWorld);
	mSoftWorld->Initialize();

	mController = Controller::Create(mSoftWorld,mRigidWorld,mMusculoSkeletalSystem);
}