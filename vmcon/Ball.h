#ifndef __BALL_H__
#define __BALL_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
class Ball
{
public:
	dart::dynamics::SkeletonPtr 				skeleton;
	dart::constraint::WeldJointConstraintPtr 	constraint;

	bool 										isReleased;

	Eigen::Vector3d								releasedPoint;
	Eigen::Vector3d								releasedVelocity;
public:
	Ball(const dart::constraint::WeldJointConstraintPtr& cons,const dart::dynamics::SkeletonPtr& skel);

	void ComputeFallingPosition(double h,Eigen::Vector3d& fp);
	void Release(const dart::simulation::WorldPtr& world);
	// void SetPosition(const Eigen::Vector3d& p);
	// void SetVelocity(const Eigen::Vector3d& v){skeleton->getBo;
	Eigen::Vector3d GetPosition();
	Eigen::Vector3d GetVelocity();
	void Attach(const dart::simulation::WorldPtr& world,dart::dynamics::BodyNode* bn);
	dart::dynamics::SkeletonPtr GetSkeleton(){return skeleton;};
	dart::constraint::WeldJointConstraintPtr GetConstraint(){return constraint;};
	bool IsReleased(){return isReleased;};
};

#endif