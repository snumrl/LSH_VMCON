#include "Controller.h"
#include "MusculoSkeletalSystem.h"
using namespace FEM;
using namespace dart::dynamics;
using namespace dart::simulation;


Controller::
Controller(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system)
	:mSoftWorld(soft_world),mRigidWorld(rigid_world),mMusculoSkeletalSystem(musculo_skeletal_system)
{
	int dof = mMusculoSkeletalSystem->GetSkeleton()->getNumDofs();
	double k = 500;

	mKp = Eigen::VectorXd::Constant(dof,k);
	mKv = Eigen::VectorXd::Constant(dof,2*sqrt(k));

	mTargetPositions = Eigen::VectorXd::Constant(dof,0.0);
	mTargetVelocities = Eigen::VectorXd::Constant(dof,0.0);
}
std::shared_ptr<Controller>
Controller::
Clone(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system)
{
	auto new_con = Create(soft_world,rigid_world,musculo_skeletal_system);

	new_con->mKp = mKp;
	new_con->mKv = mKv;

	new_con->mTargetPositions = mTargetPositions;
	new_con->mTargetVelocities = mTargetVelocities;
	return new_con;
}
std::shared_ptr<Controller>
Controller::
Create(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world,std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system)
{
	auto con = new Controller(soft_world,rigid_world,musculo_skeletal_system);

	return std::shared_ptr<Controller>(con);
}


Eigen::VectorXd
Controller::
ComputePDForces()
{
	auto& skel =mMusculoSkeletalSystem->GetSkeleton();

	Eigen::VectorXd pos_m = mTargetPositions;
	Eigen::VectorXd vel_m = mTargetVelocities;

	Eigen::VectorXd pos = skel->getPositions();
	Eigen::VectorXd vel = skel->getVelocities();

	for(int i = 0;i<pos.rows();i++)
		pos[i] = dart::math::wrapToPi(pos[i]);
	skel->setPositions(pos);
	Eigen::VectorXd pos_diff(pos.rows());

	pos_diff = skel->getPositionDifferences(pos_m,pos);
	// std::cout<<"ComputePDForces()"<<std::endl;

	std::cout<<pos.transpose()<<std::endl;
	std::cout<<pos_m.transpose()<<std::endl;
	std::cout<<pos_diff.transpose()<<std::endl;
	// for(int i =0;i<skel->getNumDofs();i++)
	// {
	// 	if(!skel->getDof(i)->getJoint()->getType().compare("RevoluteJoint"))
	// 	{
	// 		double angle = pos_m[i] - skel->getDof(i)->getPosition();
	// 		double two_phi_angle = 2*3.141592 -angle;
	// 		// std::cout<<angle<<std::endl;
	// 		// std::cout<<two_phi_angle<<std::endl;
	// 	}
	// }
	// for(int i = 0;i<pos_diff.rows();i++)
		// pos_diff[i] = dart::math::wrapToPi(pos_diff[i]);

	Eigen::VectorXd qdd_desired = 
				pos_diff.cwiseProduct(mKp)+
				(vel_m - vel).cwiseProduct(mKv);
	std::cout<<qdd_desired.transpose()<<std::endl;
	if(dart::math::isNan(pos))
		exit(0);
	return qdd_desired;
}