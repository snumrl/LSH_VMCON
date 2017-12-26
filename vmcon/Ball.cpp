#include "Ball.h"
using namespace dart::dynamics;
using namespace dart::simulation;
Ball::
Ball(const dart::constraint::WeldJointConstraintPtr& cons,const dart::dynamics::SkeletonPtr& skel)
	:isReleased(true),constraint(cons),skeleton(skel),releasedPoint(Eigen::Vector3d::Zero()),releasedVelocity(Eigen::Vector3d::Zero()),release_count(0)
{
	
}

void
Ball::
ComputeFallingPosition(double h,Eigen::Vector3d& fp)
{
	Eigen::Vector3d p = skeleton->getBodyNode(0)->getCOM();
	Eigen::Vector3d v = skeleton->getBodyNode(0)->getCOMLinearVelocity();

	double dx = h-p[1];
	double g = -9.8;
	double v2_plus_2gdx = v[1]*v[1] + 2.0*g*dx;
	if(v2_plus_2gdx<0)
	{
		// std::cout<<"no solution"<<std::endl;
		fp.setZero();
		return;
	}

	double t1 = (-v[1] - sqrt(v2_plus_2gdx))/g;
	double t2 = (-v[1] + sqrt(v2_plus_2gdx))/g;
	double t = (t1>t2?t1:t2);
	if(t<0.0)
		t=0.05;

	fp = p+t*v;
	if(fp[1]>h)
	fp[1] = h;
}
Eigen::Vector3d
Ball::
GetPosition()
{
	// std::cout<<skeleton->getBodyNode(0)->getCOM().transpose()<<std::endl;
	return skeleton->getBodyNode(0)->getCOM();
}
Eigen::Vector3d
Ball::
GetVelocity()
{
	// std::cout<<skeleton->getBodyNode(0)->getCOMLinearVelocity().transpose()<<std::endl;
	return skeleton->getBodyNode(0)->getCOMLinearVelocity();
}
void
Ball::
Release(const dart::simulation::WorldPtr& world)
{
	if(!isReleased){
		world->getConstraintSolver()->removeConstraint(constraint);
		constraint = nullptr;
		isReleased = true;
		releasedPoint = skeleton->getBodyNode(0)->getCOM();
		releasedVelocity = skeleton->getBodyNode(0)->getCOMLinearVelocity();
		release_count = 0;
	}

	
	// std::cout<<"Released Velocity : "<<releasedVelocity.transpose()<<std::endl;
}

void
Ball::
Attach(const dart::simulation::WorldPtr& world,dart::dynamics::BodyNode* bn)
{
	if(isReleased)
	{
		constraint.reset();
		constraint = std::make_shared<dart::constraint::WeldJointConstraint>(skeleton->getBodyNode(0),bn);
		isReleased = false;
		world->getConstraintSolver()->addConstraint(constraint);
	}
}
bool
Ball::
IsClose(const Eigen::Vector3d& p)
{
	Eigen::Vector3d cur_p = GetPosition();
	Eigen::Vector3d cur_v = GetVelocity();
	Eigen::Vector3d next_p = cur_p + cur_v*0.005;

	double cur_dy = cur_p[1] - p[1];
	double next_dy = next_p[1] - p[1];

	if(cur_dy<0.05)
		return true;
	else
		return false;

	// std::vector<Eigen::Vector3d> prev_p;
	// for(int i =0;i<5;i++)
	// 	prev_p.push_back(cur_p-i*cur_v);
	
	// std::vector<double> norm_p;
	// for(int i=0;i<5;i++)
	// 	norm_p.push_back((prev_p[i]-p).norm());

	// bool isMonotonic = true;
	// for(int i=0;i<3;i++)
	// {
	// 	double p01 = norm_p[i+1]-norm_p[i];
	// 	double p12 = norm_p[i+2]-norm_p[i+1];
	// 	if(p01*p12<0)
	// 		isMonotonic = false;
	// }
	// return !isMonotonic;
}