#ifndef __BALL_H__
#define __BALL_H__

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"


class Ball
{

public:
	void Release(const dart::simulation::WorldPtr& world);
	void Attach(const dart::simulation::WorldPtr& world,dart::dynamics::BodyNode* bn);
	
	Ball(const Ball& other) = delete;
	Ball& operator=(const Ball& other) = delete;
	std::shared_ptr<Ball> Clone();
	static std::shared_ptr<Ball> Create(const dart::dynamics::SkeletonPtr& skel);

	const dart::dynamics::SkeletonPtr& GetSkeleton() {return mSkeleton;};
	const dart::constraint::WeldJointConstraintPtr& GetConstraint() {return mConstraint;};


	bool IsAttached() {return isAttached;};
	std::string GetAttachedBodyName() {
		if(isAttached)
			return mConstraint->getBodyNode2()->getName();
		return "";};
private:
	bool isAttached;
	dart::dynamics::SkeletonPtr mSkeleton;
	dart::constraint::WeldJointConstraintPtr 	mConstraint;
	Ball(const dart::dynamics::SkeletonPtr& skel);
};
#endif