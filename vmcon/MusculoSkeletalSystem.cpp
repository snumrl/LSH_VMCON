#include "MusculoSkeletalSystem.h"
#include "DART_helper.h"
#include <tinyxml.h>
#include <algorithm>
using namespace FEM;
using namespace dart::dynamics;
using namespace dart::simulation;

Muscle::
Muscle()
{

}
std::shared_ptr<Muscle>
Muscle::
Clone(const FEM::WorldPtr& soft_world,const dart::dynamics::SkeletonPtr& skeleton)
{
	auto new_muscle = Create();

	new_muscle->name = name;
	auto& new_constraints = soft_world->GetConstraints();

	new_muscle->mesh = mesh->Clone();
	for(int i=0;i<origin_way_points.size();i++)
	{
		auto& ap = origin_way_points[i];
		new_muscle->origin_way_points.push_back(std::make_pair(skeleton->getBodyNode(ap.first->getName()),ap.second));
	}

	for(int i=0;i<insertion_way_points.size();i++)
	{
		auto& ap = insertion_way_points[i];
		new_muscle->insertion_way_points.push_back(std::make_pair(skeleton->getBodyNode(ap.first->getName()),ap.second));
	}

	new_muscle->activation_level = activation_level;
	new_muscle->origin_force = origin_force;
	new_muscle->insertion_force = insertion_force;

	std::cout<<(*new_constraints.begin())->GetName()<<std::endl;


	for(int i =0;i<new_constraints.size();i++)
		if(origin->Equal(new_constraints[i]))
			new_muscle->origin = std::dynamic_pointer_cast<AttachmentCst>(new_constraints[i]);
		else if(insertion->Equal(new_constraints[i]))
			new_muscle->insertion = std::dynamic_pointer_cast<AttachmentCst>(new_constraints[i]);

	new_muscle->muscle_csts.resize(muscle_csts.size());
	new_muscle->csts.resize(csts.size());
	for(int i=0;i<muscle_csts.size();i++)
	{
		for(int j =0;j<new_constraints.size();j++)
			if(muscle_csts[i]->Equal(new_constraints[j]))
				new_muscle->muscle_csts[i] = std::dynamic_pointer_cast<LinearMuscleCst>(new_constraints[j]);		
	}

	for(int i=0;i<csts.size();i++)
	{
		for(int j =0;j<new_constraints.size();j++)
			if(csts[i]->Equal(new_constraints[j]))
				new_muscle->csts[i] = new_constraints[j];		
	}

	return new_muscle;
}
std::shared_ptr<Muscle>
Muscle::
Create()
{
	auto nm = new Muscle();

	return std::shared_ptr<Muscle>(nm);
}
Eigen::Vector3d
GetPoint(const AnchorPoint& ap)
{
	return ap.first->getTransform()*ap.second;
}

void
Muscle::
TransferForce(Eigen::Vector3d& f_origin,Eigen::Vector3d& f_insertion)
{


}
void
Muscle::
SetActivationLevel(double a)
{
	for(auto& lmc : muscle_csts)
		lmc->SetActivationLevel(a);

	activation_level = a;
}

MusculoSkeletalSystem::
MusculoSkeletalSystem()
	:mTendonStiffness(1E5),mMuscleStiffness(1E6),mYoungsModulus(1E6),mPoissonRatio(0.3)
{

}
std::shared_ptr<MusculoSkeletalSystem>
MusculoSkeletalSystem::
Clone(const std::shared_ptr<FEM::World>& soft_world,const dart::simulation::WorldPtr& rigid_world)
{
	auto new_mss = Create();
	auto new_skeleton = rigid_world->getSkeleton(mSkeleton->getName());
	for(int i=0;i<mMuscles.size();i++)
		new_mss->mMuscles.push_back(mMuscles[i]->Clone(soft_world,new_skeleton));
	new_mss->mSkeleton = new_skeleton;

	new_mss->mTendonStiffness = mTendonStiffness;
	new_mss->mMuscleStiffness = mMuscleStiffness;
	new_mss->mYoungsModulus = mYoungsModulus;
	new_mss->mPoissonRatio = mPoissonRatio;
	
	new_mss->mActivationLevels = mActivationLevels;

	return new_mss;
}

std::shared_ptr<MusculoSkeletalSystem>
MusculoSkeletalSystem::
Create()
{
	auto mss = new MusculoSkeletalSystem();

	return std::shared_ptr<MusculoSkeletalSystem>(mss);
}
void
MusculoSkeletalSystem::
AddMuscle(
	const std::string& name,
	const std::vector<AnchorPoint>& origin,
	const std::vector<AnchorPoint>& insertion,
	int origin_index,int insertion_index,
	const Eigen::Vector3d& fiber_direction,
	const MeshPtr& mesh)
{
	mMuscles.push_back(Muscle::Create());
	auto& muscle = mMuscles.back();

	muscle->name = name;
	muscle->mesh = mesh;
	muscle->origin_way_points = origin;
	muscle->insertion_way_points = insertion;

	muscle->origin = AttachmentCst::Create(name+"_origin",mTendonStiffness,origin_index,GetPoint(origin[0]));
	muscle->insertion = AttachmentCst::Create(name+"_insertion",mTendonStiffness,insertion_index,GetPoint(insertion[0]));
	muscle->activation_level = 0.0;

	const auto& tetrahedrons = muscle->mesh->GetTetrahedrons();
	const auto& vertices = muscle->mesh->GetVertices();
	int tet_index = 0;
	for(const auto& tet: tetrahedrons)
	{
		int i0,i1,i2,i3;
		Eigen::Vector3d p0,p1,p2,p3;

		i0 = tet[0];
		i1 = tet[1];
		i2 = tet[2];
		i3 = tet[3];

		p0 = vertices[i0];
		p1 = vertices[i1];
		p2 = vertices[i2];
		p3 = vertices[i3];

		Eigen::Matrix3d Dm;

		Dm.block<3,1>(0,0) = p1 -p0;
		Dm.block<3,1>(0,1) = p2 -p0;
		Dm.block<3,1>(0,2) = p3 -p0;

		//Peventing inversion
		if(Dm.determinant()<0)
		{
			i2 = tet[3];
			i3 = tet[2];
			
			p2 = vertices[i2];
			p3 = vertices[i3];

			Dm.block<3,1>(0,1) = p2-p0;
			Dm.block<3,1>(0,2) = p3-p0;
		}

		muscle->muscle_csts.push_back(LinearMuscleCst::Create(name+"_muscle_"+std::to_string(tet_index),
			mMuscleStiffness,
			i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse(),
			fiber_direction));
		muscle->csts.push_back(CorotateFEMCst::Create(name+"_element_"+std::to_string(tet_index),
			mYoungsModulus,
			mPoissonRatio,
			i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse()));

		tet_index++;
	}
	for(auto c: muscle->muscle_csts)
		muscle->csts.push_back(c);
	muscle->csts.push_back(muscle->origin);
	muscle->csts.push_back(muscle->insertion);
}
void
MusculoSkeletalSystem::
Initialize(const std::shared_ptr<FEM::World>& soft_world,const dart::simulation::WorldPtr& rigid_world)
{
	
	for(int i =0;i<mMuscles.size();i++)
	{
		int offset = soft_world->GetNumVertices();
		auto& muscle = mMuscles[i];

		const auto& vertices = muscle->mesh->GetVertices();

		for(auto& c: muscle->csts)
			c->AddOffset(offset);
		Eigen::VectorXd v(vertices.size()*3);
		for(int i =0;i<vertices.size();i++)
			v.block<3,1>(i*3,0) = vertices[i];

		soft_world->AddBody(v,muscle->csts,1.0);
	}

	mActivationLevels.resize(mMuscles.size());
	mActivationLevels.setZero();

	rigid_world->addSkeleton(mSkeleton);
}
void
MusculoSkeletalSystem::
SetActivationLevels(const Eigen::VectorXd& a)
{
	mActivationLevels = a;
	for(int i =0;i<mMuscles.size();i++)
		mMuscles[i]->SetActivationLevel(a[i]);
}
void
MusculoSkeletalSystem::
TransformAttachmentPoints()
{
	for(auto& muscle : mMuscles)
	{
		auto& origin_way_points = muscle->origin_way_points;
		auto& insertion_way_points = muscle->insertion_way_points;

		Eigen::Vector3d po = GetPoint(origin_way_points[0]);
		Eigen::Vector3d pi = GetPoint(insertion_way_points[0]);

		muscle->origin->SetP(po);
		muscle->insertion->SetP(pi);
	}
}
void
MusculoSkeletalSystem::
ApplyForcesToSkeletons(const std::shared_ptr<FEM::World>& soft_world)
{
	Eigen::VectorXd X = soft_world->GetPositions();
	Eigen::VectorXd force_origin(X.rows()),force_insertion(X.rows());
	Eigen::Vector3d fo,fi;

	for(auto& muscle: mMuscles)
	{
		auto& origin_way_points = muscle->origin_way_points;
		auto& insertion_way_points = muscle->insertion_way_points;

		int no = origin_way_points.size();
		int ni = insertion_way_points.size();

		force_origin.setZero();
		force_insertion.setZero();

		muscle->origin->EvaluateGradient(X);
		muscle->insertion->EvaluateGradient(X);

		muscle->origin->GetGradient(force_origin);
		muscle->insertion->GetGradient(force_insertion);

		fo = force_origin.block<3,1>(muscle->origin->GetI0()*3,0);
		fi = force_insertion.block<3,1>(muscle->insertion->GetI0()*3,0);
		muscle->TransferForce(fo,fi);

		origin_way_points.back().first->addExtForce(fo,origin_way_points.back().second);
		insertion_way_points.back().first->addExtForce(fi,insertion_way_points.back().second);

		muscle->origin_force = fo;
		muscle->insertion_force = fi;
	}

}

void MakeMuscles(const std::string& path,std::shared_ptr<MusculoSkeletalSystem>& ms)
{
	auto& skel = ms->GetSkeleton();
	TiXmlDocument doc;
    if(!doc.LoadFile(path))
    {
        std::cout<<"Cant open XML file : "<<path<<std::endl;
        return;
    }

    TiXmlElement* muscles = doc.FirstChildElement("Muscles");

    for(TiXmlElement* unit = muscles->FirstChildElement("unit");unit!=nullptr;unit = unit->NextSiblingElement("unit"))
    {
        TiXmlElement* ori = unit->FirstChildElement("origin");
        std::string name = (unit->Attribute("name"));
        std::vector<AnchorPoint> p_ori,p_ins;
       
        for(TiXmlElement* anc = ori->FirstChildElement("anchor");anc!=nullptr;anc = anc->NextSiblingElement("anchor"))   
        {
            std::string body_name = anc->Attribute("body");
            double x = std::stod(anc->Attribute("x"));
            double y = std::stod(anc->Attribute("y"));
            double z = std::stod(anc->Attribute("z"));
            p_ori.push_back(AnchorPoint(skel->getBodyNode(body_name.c_str()),Eigen::Vector3d(x,y,z)));
        }
        
        TiXmlElement* ins = unit->FirstChildElement("insertion");
        for(TiXmlElement* anc = ins->FirstChildElement("anchor");anc!=nullptr;anc = anc->NextSiblingElement("anchor"))   
        {
            std::string body_name = anc->Attribute("body");
            double x = std::stod(anc->Attribute("x"));
            double y = std::stod(anc->Attribute("y"));
            double z = std::stod(anc->Attribute("z"));
            p_ins.push_back(AnchorPoint(skel->getBodyNode(body_name.c_str()),Eigen::Vector3d(x,y,z)));
        }

        Eigen::Vector3d muscle_start,muscle_end;

        muscle_start = GetPoint(p_ori[0]);
        muscle_end = GetPoint(p_ins[0]);

        double len = (muscle_start - muscle_end).norm();
        Eigen::Vector3d unit_dir = (muscle_start - muscle_end).normalized();

        Eigen::Vector3d axis = Eigen::Vector3d::UnitX().cross(unit_dir);
        double cos_angle = unit_dir[0];
        double sin_angle = axis.norm();

        double angle = atan2(sin_angle,cos_angle);
        TiXmlElement* mesh_element = unit->FirstChildElement("mesh");

        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translation() = 0.5*(muscle_start + muscle_end);
        T.linear() = len*(Eigen::AngleAxisd(angle,axis.normalized()).matrix());
        int nx = std::stoi(mesh_element->Attribute("nx"));
        int ny = std::stoi(mesh_element->Attribute("ny"));
        double ratio = std::stod(mesh_element->Attribute("ratio"));
        auto dm = DiamondMesh::Create(1.0,(double)ny/(double)nx*ratio,(double)ny/(double)nx*ratio,nx,ny,ny,T);
        int i_ori = dm->GetEndingPointIndex();
        int i_ins = dm->GetStartingPointIndex();

        ms->AddMuscle(name,p_ori,p_ins,i_ori,i_ins,unit_dir,dm);
        
        
    }
}
void MakeSkeleton(std::shared_ptr<MusculoSkeletalSystem>& ms)
{
	ms->GetSkeleton() = Skeleton::create("HUMAN");
	auto& skel = ms->GetSkeleton();

	MakeRootBody(skel,"Torso",
		Eigen::Vector3d(0.03,0.6,0.03),
		Eigen::Vector3d(0,-0.3,0),
		JOINT_TYPE::EULER,10);
	
	MakeBody(skel,skel->getBodyNode("Torso"),"NeckR",
		Eigen::Vector3d(0.3,0.03,0.03),
		Eigen::Vector3d(0.0,0.3,0),
		Eigen::Vector3d(-0.15,0.0,0),JOINT_TYPE::EULER,5);

	MakeBody(skel,skel->getBodyNode("Torso"),"NeckL",
		Eigen::Vector3d(0.3,0.03,0.03),
		Eigen::Vector3d(0.0,0.3,0),
		Eigen::Vector3d(0.15,0.0,0),JOINT_TYPE::EULER,5);

	MakeBody(skel,skel->getBodyNode("NeckR"),"ShoulderR",
		Eigen::Vector3d(0.3,0.03,0.03),
		Eigen::Vector3d(0.15,0.0,0),
		Eigen::Vector3d(-0.15,0.0,0),JOINT_TYPE::EULER,5);

	MakeBody(skel,skel->getBodyNode("NeckL"),"ShoulderL",
		Eigen::Vector3d(0.3,0.03,0.03),
		Eigen::Vector3d(-0.15,0.0,0),
		Eigen::Vector3d(0.15,0.0,0),JOINT_TYPE::EULER,5);

	MakeBody(skel,skel->getBodyNode("ShoulderR"),"ElbowR",
		Eigen::Vector3d(0.3,0.03,0.03),
		Eigen::Vector3d(0.15,0.0,0),
		Eigen::Vector3d(-0.15,0.0,0),JOINT_TYPE::REVOLUTE,5);

	MakeBody(skel,skel->getBodyNode("ShoulderL"),"ElbowL",
		Eigen::Vector3d(0.3,0.03,0.03),
		Eigen::Vector3d(-0.15,0.0,0),
		Eigen::Vector3d(0.15,0.0,0),JOINT_TYPE::REVOLUTE,5);

	MakeBody(skel,skel->getBodyNode("Torso"),"Head",
		Eigen::Vector3d(0.07,0.07,0.07),
		Eigen::Vector3d(0,0.40,0),
		Eigen::Vector3d(0,0,0),
		JOINT_TYPE::WELD,
		10);

	MakeBody(skel,skel->getBodyNode("ElbowR"),"HandR",
		Eigen::Vector3d(0.07,0.07,0.07),
		Eigen::Vector3d(0.17,0,0),
		Eigen::Vector3d(0,0,0),
		JOINT_TYPE::WELD,
		3);

	MakeBody(skel,skel->getBodyNode("ElbowR"),"ThumbR",
		Eigen::Vector3d(0.03,0.03,0.03),
		Eigen::Vector3d(0.17,0.05,0),
		Eigen::Vector3d(0,0,0),
		JOINT_TYPE::WELD,
		3);

	MakeBody(skel,skel->getBodyNode("ElbowL"),"HandL",
		Eigen::Vector3d(0.07,0.07,0.07),
		Eigen::Vector3d(-0.17,0.0,0),
		Eigen::Vector3d(0,0,0),
		JOINT_TYPE::WELD,
		3);

	MakeBody(skel,skel->getBodyNode("ElbowL"),"ThumbL",
		Eigen::Vector3d(0.03,0.03,0.03),
		Eigen::Vector3d(-0.17,0.05,0),
		Eigen::Vector3d(0,0,0),
		JOINT_TYPE::WELD,
		3);
	Eigen::VectorXd pos = skel->getPositions();

	pos[3*1+1] = -0.1;
	pos[3*2+1] = 0.1;

	pos[3*1+2] = 0.1;
	pos[3*2+2] = -0.1;

	pos[3*3+0] = 0.3;
	pos[3*4+0] = 0.3;

	pos[3*3+1] = -0.7;
	pos[3*4+1] = 0.7;

	pos[3*3+2] = -1.0;
	pos[3*4+2] = 1.0;

	//Revolute Joint
	pos[3*5] = -0.5;
	pos[3*5+1] = 0.5;

	//Euler JOint
	// pos[3*5+0] = -1.0;
	// pos[3*6+0] = -1.0;
	skel->setPositions(pos);
	skel->computeForwardKinematics(true,false,false);
}