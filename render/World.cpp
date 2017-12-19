#include "World.h"
#include "../vmcon/DART_helper.h"
#include "../vmcon/MusculoSkeletalSystem.h"
#include "../vmcon/Ball.h"
#include "Record.h"
#include <tinyxml.h>

using namespace dart::dynamics;
using namespace dart::simulation;


IntegratedWorld::
IntegratedWorld(const std::string& path)
{
	Initialize(path);
}


void
IntegratedWorld::
Initialize(const std::string& path)
{
/*	TiXmlDocument doc;
    if(!doc.LoadFile(path))
    {
        std::cout<<"Cant open XML file : "<<path<<std::endl;
        return;
    }

    TiXmlElement* soft_world = doc.FirstChildElement("Soft");
    TiXmlElement* nodes_elem = soft_world->FirstChildElement("Nodes");
    TiXmlElement* constraints_elem = soft_world->FirstChildElement("Constraints");

    std::vector<FEM::CstPtr> constraints;
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
    for(TiXmlElement* con_elem = constraints_elem->FirstChildElement("constraint");con_elem!=nullptr;con_elem = con_elem->NextSiblingElement("constraint"))
    {
    	std::string type = con_elem->Attribute("type");
    	if(!type.compare("attachment"))
    	{
    		int i0 = std::stoi(con_elem->Attribute("i0"));
			constraints.push_back(FEM::AttachmentCst::Create("",1,i0,Eigen::Vector3d(0,0,0)));

    	}
    	else if(!type.compare("corotate"))
    	{
    		int i0 = std::stoi(con_elem->Attribute("i0"));
    		int i1 = std::stoi(con_elem->Attribute("i1"));
    		int i2 = std::stoi(con_elem->Attribute("i2"));
    		int i3 = std::stoi(con_elem->Attribute("i3"));	
    		constraints.push_back(FEM::CorotateFEMCst::Create("",1,1,i0,i1,i2,i3,1,M));
    	}
    	else if(!type.compare("muscle"))
    	{
    		int i0 = std::stoi(con_elem->Attribute("i0"));
    		int i1 = std::stoi(con_elem->Attribute("i1"));
    		int i2 = std::stoi(con_elem->Attribute("i2"));
    		int i3 = std::stoi(con_elem->Attribute("i3"));
    		double dir_x = std::stoi(con_elem->Attribute("dx"));
    		double dir_y = std::stoi(con_elem->Attribute("dy"));
    		double dir_z = std::stoi(con_elem->Attribute("dz"));

    		Eigen::Vector3d dir = Eigen::Vector3d(dir_x,dir_y,dir_z);
    		
    		constraints.push_back(FEM::LinearMuscleCst::Create("",1,i0,i1,i2,i3,1,M,dir));
    	}
    }


    std::vector<Eigen::Vector3d> vertex;
    for(TiXmlElement* node_elem = nodes_elem->FirstChildElement("node");node_elem!=nullptr;node_elem = node_elem->NextSiblingElement("node"))
    {
    	double x = std::stod(node_elem->Attribute("x"));
    	double y = std::stod(node_elem->Attribute("y"));
    	double z = std::stod(node_elem->Attribute("z"));

    	vertex.push_back(Eigen::Vector3d(x,y,z));
    }
    Eigen::VectorXd x0(vertex.size()*3);

    for(int i=0;i<vertex.size();i++)
    {
    	x0.block<3,1>(i*3,0) = vertex[i];
    }
    mSoftWorld = FEM::World::Create();
    mSoftWorld->AddBody(x0,constraints);
*/
    mSoftWorld = FEM::World::Create(
		FEM::IntegrationMethod::PROJECTIVE_QUASI_STATIC,	//Integration Method
		// FEM::IntegrationMethod::QUASI_STATIC,	//Integration Method
		// FEM::IntegrationMethod::PROJECTIVE_DYNAMICS,	//Integration Method
		1.0/200.0,							//Time Step
		20,								//Max Iteration
		Eigen::Vector3d(0,-9.81,0),					//Gravity
		0.999								//Damping
		);
	mRigidWorld = std::make_shared<dart::simulation::World>();
	mRigidWorld->setGravity(Eigen::Vector3d(0,-9.81,0));
	mRigidWorld->checkCollision();

	mRigidWorld = std::make_shared<dart::simulation::World>();
	mMusculoSkeletalSystem = MusculoSkeletalSystem::Create();
    MakeSkeleton(mMusculoSkeletalSystem);
	MakeMuscles("../vmcon/export/muscle_params.xml",mMusculoSkeletalSystem);

	mMusculoSkeletalSystem->Initialize(mSoftWorld,mRigidWorld);
	mSoftWorld->Initialize();
	MakeBalls(mRigidWorld,mMusculoSkeletalSystem,mBalls,5);
}

