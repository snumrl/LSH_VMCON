#include "DART_helper.h"


using namespace dart::dynamics;
using namespace dart::simulation;

void
MakeRootBody(
	const dart::dynamics::SkeletonPtr& skel,
	const std::string& name,
	const std::string& obj_path,
	const Eigen::Isometry3d& visual_T,
	const Eigen::Vector3d& size,
	const Eigen::Vector3d& c_to_joint,
	JOINT_TYPE joint_type,
	double mass)
{
	ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(size));

	char* curi;
    curi = realpath(obj_path.c_str(), NULL);
    const aiScene* mesh = MeshShape::loadMesh(std::string(curi));
    ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(0.01,0.01,0.01),mesh,obj_path));

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
	auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
	auto sn = bn->createShapeNodeWith<VisualAspect,CollisionAspect, DynamicsAspect>(shape);

	// auto sn = bn->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);
	vsn->setRelativeTransform(visual_T);
	bn->setInertia(inertia);
}

void
MakeBody(
	const dart::dynamics::SkeletonPtr& skel,
	const dart::dynamics::BodyNodePtr& parent,
	const std::string& name,
	const std::string& obj_path,
	const Eigen::Isometry3d& visual_T,
	const Eigen::Vector3d& size,
	const Eigen::Vector3d& p_to_joint,
	const Eigen::Vector3d& c_to_joint,
	JOINT_TYPE joint_type,
	double mass)
{
		ShapePtr shape = std::shared_ptr<BoxShape>(new BoxShape(size));
		char* curi;
    curi = realpath(obj_path.c_str(), NULL);
    const aiScene* mesh = MeshShape::loadMesh(std::string(curi));
    ShapePtr visual_shape = std::shared_ptr<MeshShape>(new MeshShape(Eigen::Vector3d(0.01,0.01,0.01),mesh,obj_path));

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
	// auto sn = bn->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);

	auto vsn = bn->createShapeNodeWith<VisualAspect>(visual_shape);
	auto sn = bn->createShapeNodeWith<VisualAspect,CollisionAspect, DynamicsAspect>(shape);

	vsn->setRelativeTransform(visual_T);

	bn->setInertia(inertia);
}
void MakeBall(
	const dart::dynamics::SkeletonPtr& skel,
	const Eigen::Vector3d& init_pos,
	double rad,
	double mass)
{
	ShapePtr shape = std::shared_ptr<SphereShape>(new SphereShape(rad));
    dart::dynamics::Inertia inertia;
    inertia.setMass(mass);
    inertia.setMoment(shape->computeInertia(mass));

    FreeJoint::Properties prop;
    prop.mT_ParentBodyToJoint.setIdentity();
    prop.mT_ChildBodyToJoint.setIdentity();

    BodyNodePtr bn = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr,prop,BodyNode::AspectProperties("ball")).second;

    auto sn = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(shape);
    bn->setCollidable(false);
    bn->setInertia(inertia);
    auto pos =skel->getPositions();
    pos.tail(3) = init_pos;
    skel->setPositions(pos);
    for(int i=0;i<skel->getNumBodyNodes();i++)
		skel->getBodyNode(i)->setCollidable(false);
}
