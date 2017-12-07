
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

	Eigen::VectorXd								initialPosition;
public:
	Ball(const dart::constraint::WeldJointConstraintPtr& cons,const dart::dynamics::SkeletonPtr& skel);

	void ComputeFallingPosition(double h,Eigen::Vector3d& fp);
	void Attach(const dart::simulation::WorldPtr& world,dart::dynamics::BodyNode* bn);
	void Release(const dart::simulation::WorldPtr& world);
	void Reset(const dart::simulation::WorldPtr& world);

	Eigen::Vector3d GetPosition();
	Eigen::Vector3d GetVelocity();


};

#endif