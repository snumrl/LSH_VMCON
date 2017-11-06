#include "DART_helper.h"


using namespace dart::dynamics;
using namespace dart::simulation;

void
MakeRootBody(
	const dart::dynamics::SkeletonPtr& skel,
	const std::string& name,
	const Eigen::Vector3d& size,
	const Eigen::Vector3d& c_to_joint,
	JOINT_TYPE joint_type,
	double mass)
{
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(size));
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	BodyNodePtr bn;
	if(joint_type == JOINT_TYPE::WELD)
	{
		WeldJoint::Properties prop;
		prop.mName = name + "_joint";
		prop.mT_ChildBodyToJoint.translation() = c_to_joint;

		bn = skel->createJointAndBodyNodePair<WeldJoint>(
			nullptr,prop,BodyNode::AspectProperties(name)).second;	
	}
	else if(joint_type == JOINT_TYPE::REVOLUTE)
	{
		RevoluteJoint::Properties prop;
		prop.mName = name + "_joint";
		prop.mAxis = Eigen::Vector3d::UnitZ();
		prop.mT_ChildBodyToJoint.translation() = c_to_joint;

		bn = skel->createJointAndBodyNodePair<RevoluteJoint>(
			nullptr,prop,BodyNode::AspectProperties(name)).second;	
	}
	else if(joint_type == JOINT_TYPE::UNIVERSAL)
	{
		UniversalJoint::Properties prop;
		prop.mName = name + "_joint";
		prop.mAxis[0] = Eigen::Vector3d::UnitY();
		prop.mAxis[1] = Eigen::Vector3d::UnitZ();
		prop.mT_ChildBodyToJoint.translation() = c_to_joint;

		bn = skel->createJointAndBodyNodePair<UniversalJoint>(
			nullptr,prop,BodyNode::AspectProperties(name)).second;	
	}
	else if(joint_type == JOINT_TYPE::EULER)
	{
		EulerJoint::Properties prop;
		prop.mAxisOrder = dart::dynamics::detail::AxisOrder::XYZ;
		prop.mName = name + "_joint";
		prop.mT_ChildBodyToJoint.translation() = c_to_joint;

		bn = skel->createJointAndBodyNodePair<EulerJoint>(
			nullptr,prop,BodyNode::AspectProperties(name)).second;
	}
	else if(joint_type == JOINT_TYPE::BALL_AND_SOCKET)
	{	
		BallJoint::Properties prop;
		prop.mName = name + "_joint";
		prop.mT_ChildBodyToJoint.translation() = c_to_joint;

		bn = skel->createJointAndBodyNodePair<BallJoint>(
			nullptr,prop,BodyNode::AspectProperties(name)).second;	
	}
	auto sn = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);
	bn->setInertia(inertia);
}

void
MakeBody(
	const dart::dynamics::SkeletonPtr& skel,
	const dart::dynamics::BodyNodePtr& parent,
	const std::string& name,
	const Eigen::Vector3d& size,
	const Eigen::Vector3d& p_to_joint,
	const Eigen::Vector3d& c_to_joint,
	JOINT_TYPE joint_type,
	double mass)
{
		ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(size));
	dart::dynamics::Inertia inertia;
	inertia.setMass(mass);
	inertia.setMoment(shape->computeInertia(mass));

	BodyNodePtr bn;
	if(joint_type == JOINT_TYPE::WELD)
	{
		WeldJoint::Properties prop;
		prop.mName = name + "_joint";
    	prop.mT_ParentBodyToJoint.translation() = p_to_joint;
		prop.mT_ChildBodyToJoint.translation() = c_to_joint;

		bn = skel->createJointAndBodyNodePair<WeldJoint>(
			parent,prop,BodyNode::AspectProperties(name)).second;	
	}
	else if(joint_type == JOINT_TYPE::REVOLUTE)
	{
		RevoluteJoint::Properties prop;
		prop.mName = name + "_joint";
		prop.mAxis = Eigen::Vector3d::UnitY();
    	prop.mT_ParentBodyToJoint.translation() = p_to_joint;
		prop.mT_ChildBodyToJoint.translation() = c_to_joint;

		bn = skel->createJointAndBodyNodePair<RevoluteJoint>(
			parent,prop,BodyNode::AspectProperties(name)).second;	
	}
	else if(joint_type == JOINT_TYPE::UNIVERSAL)
	{
		UniversalJoint::Properties prop;
		prop.mName = name + "_joint";
		prop.mAxis[0] = Eigen::Vector3d::UnitY();
		prop.mAxis[1] = Eigen::Vector3d::UnitZ();
    	prop.mT_ParentBodyToJoint.translation() = p_to_joint;
		prop.mT_ChildBodyToJoint.translation() = c_to_joint;

		bn = skel->createJointAndBodyNodePair<UniversalJoint>(
			parent,prop,BodyNode::AspectProperties(name)).second;	
	}
	else if(joint_type == JOINT_TYPE::EULER)
	{
		EulerJoint::Properties prop;
		prop.mAxisOrder = dart::dynamics::detail::AxisOrder::XYZ;
		prop.mName = name + "_joint";
    	prop.mT_ParentBodyToJoint.translation() = p_to_joint;
		prop.mT_ChildBodyToJoint.translation() = c_to_joint;

		bn = skel->createJointAndBodyNodePair<EulerJoint>(
			parent,prop,BodyNode::AspectProperties(name)).second;
	}
	else if(joint_type == JOINT_TYPE::BALL_AND_SOCKET)
	{	
		BallJoint::Properties prop;
		prop.mName = name + "_joint";
    	prop.mT_ParentBodyToJoint.translation() = p_to_joint;
		prop.mT_ChildBodyToJoint.translation() = c_to_joint;

		bn = skel->createJointAndBodyNodePair<BallJoint>(
			parent,prop,BodyNode::AspectProperties(name)).second;	
	}
	auto sn = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);
	bn->setInertia(inertia);
}
