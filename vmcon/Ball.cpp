#include "Ball.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::constraint;

Ball::
Ball(const SkeletonPtr& skel)
	:isAttached(false),mSkeleton(skel),mConstraint(nullptr)
{

}
std::shared_ptr<Ball>
Ball::
Clone()
{
	auto new_ball = Create(this->mSkeleton);

	new_ball->mSkeleton = mSkeleton->clone();
	// new_ball->mConstraint = std::make_shared<WeldJointConstraint>(new_ball->mSkeleton->getBodyNode(0), mConstraint->getBodyNode2());
	return new_ball;
}
std::shared_ptr<Ball>
Ball::
Create(const SkeletonPtr& skel)
{
	auto ball = new Ball(skel);
	return std::shared_ptr<Ball>(ball);
}
void
Ball::
Release(const dart::simulation::WorldPtr& world)
{
	isAttached = false;
	world->getConstraintSolver()->removeConstraint(mConstraint);
}
void
Ball::
Attach(const WorldPtr& world,BodyNode* bn)
{
	isAttached = true;
	mConstraint = std::make_shared<WeldJointConstraint>(mSkeleton->getBodyNode(0),bn);
	world->getConstraintSolver()->addConstraint(mConstraint);
}