#include "World.h"
#include "DART_helper.h"
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
	TiXmlDocument doc;
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

	mRigidWorld = std::make_shared<dart::simulation::World>();
    MakeSkeleton(mRigidWorld);
    MakeBalls(mRigidWorld);
	// mRigidWorld = std::make_shared<dart::simulation::World>();
 //    TiXmlDocument* rigid_world = doc.FirstChildElement("Rigid");
 //    for(TiXmlElement* body_elem = rigid_world->FirstChildElement("skeleton");body_elem!=nullptr;body_elem = body_elem->NextSiblingElement("skeleton"))
 //    {
 //    }
}

void MakeSkeleton(dart::simulation::WorldPtr& world)
{
	auto skel = Skeleton::create("HUMAN");
	std::string path_export = "../vmcon/export/skeleton/";
	Eigen::Isometry3d T_torso;
	Eigen::Isometry3d T_neckL;
	Eigen::Isometry3d T_neckR;
	Eigen::Isometry3d T_ShoulderL;
	Eigen::Isometry3d T_ShoulderR;
	Eigen::Isometry3d T_ElbowL;
	Eigen::Isometry3d T_ElbowR;
	Eigen::Isometry3d T_HandR;
	Eigen::Isometry3d T_HandL;

	T_torso.setIdentity();
	T_neckL.setIdentity();
	T_neckR.setIdentity();
	T_ShoulderL.setIdentity();
	T_ShoulderR.setIdentity();
	T_ElbowL.setIdentity();
	T_ElbowR.setIdentity();
	
	T_HandR.setIdentity();
	T_HandL.setIdentity();
	T_torso.translation() = Eigen::Vector3d(0,-1.3,0);
	
	MakeRootBody(skel,"Torso",
		path_export+"Torso.obj",
		T_torso,
		Eigen::Vector3d(0.03,0.6,0.03),
		Eigen::Vector3d(0,-0.3,0),
		JOINT_TYPE::BALL_AND_SOCKET,10);


	T_neckL.translation() = Eigen::Vector3d(-0.1,-1.45,0);
	T_neckR.translation() = Eigen::Vector3d(0.1,-1.45,0);
	MakeBody(skel,skel->getBodyNode("Torso"),"NeckL",
		path_export+"NeckL.obj",
		T_neckL,
		Eigen::Vector3d(0.2,0.03,0.03),
		Eigen::Vector3d(0.0,0.15,0),
		Eigen::Vector3d(-0.1,0.0,0),JOINT_TYPE::REVOLUTE,5);

	MakeBody(skel,skel->getBodyNode("Torso"),"NeckR",
		path_export+"NeckR.obj",
		T_neckR,
		Eigen::Vector3d(0.2,0.03,0.03),
		Eigen::Vector3d(0.0,0.15,0),
		Eigen::Vector3d(0.1,0.0,0.0),JOINT_TYPE::REVOLUTE,5);


	T_ShoulderR.linear() =	(Eigen::AngleAxisd(3.141592*0.47,Eigen::Vector3d(0,0,1)).toRotationMatrix()*
					Eigen::AngleAxisd(-3.141592*0.05,Eigen::Vector3d(0,1,0)).toRotationMatrix()).transpose();
	T_ShoulderR.translation() = Eigen::Vector3d(-1.24,-0.34,0.25);

	T_ShoulderL.linear() = Eigen::AngleAxisd(3.141592*0.47,Eigen::Vector3d(0,0,1)).toRotationMatrix()*
					Eigen::AngleAxisd(-3.141592*0.05,Eigen::Vector3d(1,0,0)).toRotationMatrix();
	T_ShoulderL.translation() = Eigen::Vector3d(1.24,-0.34,0.25);


	MakeBody(skel,skel->getBodyNode("NeckL"),"ShoulderL",
		path_export+"ShoulderL.obj",
		T_ShoulderL,
		Eigen::Vector3d(0.3,0.03,0.03),
		Eigen::Vector3d(0.08,-0.03,-0.03),
		Eigen::Vector3d(-0.15,0.0,0),JOINT_TYPE::BALL_AND_SOCKET,5);

	MakeBody(skel,skel->getBodyNode("NeckR"),"ShoulderR",
		path_export+"ShoulderR.obj",
		T_ShoulderR,
		Eigen::Vector3d(0.3,0.03,0.03),
		Eigen::Vector3d(-0.08,-0.03,-0.03),
		Eigen::Vector3d(0.15,0.0,0),JOINT_TYPE::BALL_AND_SOCKET,5);



	T_ElbowR.linear() =	
						Eigen::AngleAxisd(-3.141592*0.1,Eigen::Vector3d(0,1,0)).toRotationMatrix()*
						Eigen::AngleAxisd(-3.141592*0.5,Eigen::Vector3d(0,0,1)).toRotationMatrix()
						;
					// Eigen::AngleAxisd(-3.141592*0.05,Eigen::Vector3d(0,1,0)).toRotationMatrix()).inverse();
	T_ElbowR.translation() = Eigen::Vector3d(-0.93,-0.24,-0.28);

	T_ElbowL.linear() = 
					Eigen::AngleAxisd(3.141592*0.1,Eigen::Vector3d(0,1,0)).toRotationMatrix()*
					Eigen::AngleAxisd(3.141592*0.5,Eigen::Vector3d(0,0,1)).toRotationMatrix()
					;
	T_ElbowL.translation() = Eigen::Vector3d(0.93,-0.24,-0.28);

	MakeBody(skel,skel->getBodyNode("ShoulderL"),"ElbowL",
		path_export+"ElbowL.obj",
		T_ElbowL,
		Eigen::Vector3d(0.3,0.03,0.03),
		Eigen::Vector3d(0.18,0.0,0.02),
		Eigen::Vector3d(-0.15,0.0,0),JOINT_TYPE::REVOLUTE,5);

	MakeBody(skel,skel->getBodyNode("ShoulderR"),"ElbowR",
		path_export+"ElbowR.obj",
		T_ElbowR,
		Eigen::Vector3d(0.3,0.03,0.03),
		Eigen::Vector3d(-0.18,0.0,0.02),
		Eigen::Vector3d(0.15,0.0,0),JOINT_TYPE::REVOLUTE,5);




	T_torso.translation() = Eigen::Vector3d(0,-1.6,0);
	MakeBody(skel,skel->getBodyNode("Torso"),"Head",
		path_export+"Head.obj",
		T_torso,
		Eigen::Vector3d(0.07,0.07,0.07),
		Eigen::Vector3d(0,0.33,0),
		Eigen::Vector3d(0,0,0),
		JOINT_TYPE::WELD,
		10);

	T_HandR.linear() = 
					Eigen::AngleAxisd(-3.141592*0.1,Eigen::Vector3d(0,1,0)).toRotationMatrix()*
					Eigen::AngleAxisd(-3.141592*0.5,Eigen::Vector3d(0,0,1)).toRotationMatrix()
					;
	T_HandR.translation() = Eigen::Vector3d(-0.77,-0.24,-0.28);

	T_HandL.linear() = 
					Eigen::AngleAxisd(3.141592*0.1,Eigen::Vector3d(0,1,0)).toRotationMatrix()*
					Eigen::AngleAxisd(3.141592*0.5,Eigen::Vector3d(0,0,1)).toRotationMatrix()
					;
	T_HandL.translation() = Eigen::Vector3d(0.77,-0.24,-0.28);
	MakeBody(skel,skel->getBodyNode("ElbowR"),"HandR",
		path_export+"HandR.obj",
		T_HandR,
		Eigen::Vector3d(0.07,0.07,0.07),
		Eigen::Vector3d(-0.17,0,0),
		Eigen::Vector3d(0,0,0),
		JOINT_TYPE::REVOLUTE,
		3);

	MakeBody(skel,skel->getBodyNode("ElbowL"),"HandL",
		path_export+"HandL.obj",
		T_HandL,
		Eigen::Vector3d(0.07,0.07,0.07),
		Eigen::Vector3d(0.17,0.0,0),
		Eigen::Vector3d(0,0,0),
		JOINT_TYPE::REVOLUTE,
		3);

	Eigen::VectorXd pos = skel->getPositions();

	//Revolute Joints
	pos[3*1+0] = -0.1;
	pos[3*1+1] = 0.1;

	//Ball Joints

	pos[3*3+0-4] = 0.3;
	pos[3*4+0-4] = 0.3;

	pos[3*3+1-4] = -0.7;
	pos[3*4+1-4] = 0.7;

	pos[3*3+2-4] = -1.0;
	pos[3*4+2-4] = 1.0;

	// //Revolute Joint

	pos[3*5-4] = -0.5;
	pos[3*5+1-4] = 0.5;

	//Root joint

	skel->getDof(3*0+0)->setPositionLimits(-0.0,0.0);
	skel->getDof(3*0+1)->setPositionLimits(-0.1,0.1);
	skel->getDof(3*0+2)->setPositionLimits(-0.1,0.1);

	// //Revolute Joint
	skel->getDof(3*1+0)->setPositionLimits(-0.3,0.2);
	skel->getDof(3*1+1)->setPositionLimits(-0.2,0.3);

	// skel->getDof(3*1+0)->setPositionLimits(0.0,0.0);
	// skel->getDof(3*1+1)->setPositionLimits(0.0,0.0);

	// //Shoulder JOint : Euler
	skel->getDof(3*3+0-4)->setPositionLimits(0.0,1.57); //X
	skel->getDof(3*3+1-4)->setPositionLimits(-2.0,0.2); //Y
	skel->getDof(3*3+2-4)->setPositionLimits(-1.8,0.75); //Z

	skel->getDof(3*4+0-4)->setPositionLimits(0.0,1.57); //X
	skel->getDof(3*4+1-4)->setPositionLimits(-0.2,2.0); //Y
	skel->getDof(3*4+2-4)->setPositionLimits(-0.75,1.8); //Z

	skel->getDof(3*5+0-4)->setPositionLimits(-2.2,0.1); 
	skel->getDof(3*5+1-4)->setPositionLimits(0.1,2.2); 

	for(int i =0;i<skel->getNumDofs();i++){
	// 	skel->getDof(i)->setPositionLimits(-100,100);
		skel->getDof(i)->getJoint()->setPositionLimitEnforced(true);
	}
	for(int i=0;i<skel->getNumBodyNodes();i++)
		skel->getBodyNode(i)->setCollidable(false);
	//Euler JOint
	// pos[3*5+0] = -1.0;
	// pos[3*6+0] = -1.0;
	skel->setPositions(pos);
	skel->computeForwardKinematics(true,false,false);

	world->addSkeleton(skel);
}
void MakeBalls(dart::simulation::WorldPtr& world)
{
	for(int i =0;i<3;i++)
	{
		SkeletonPtr skel = Skeleton::create("ball_"+std::to_string(i));

		bool is_left_hand = i%2;
		if(is_left_hand)
		{
			auto* abn = world->getSkeleton(0)->getBodyNode("HandL");
			Eigen::Vector3d loc = abn->getTransform().translation();
			MakeBall(skel,abn->getCOM(),0.036,0.13);

			world->addSkeleton(skel);
		}
		else
		{
			auto* abn =world->getSkeleton(0)->getBodyNode("HandR");
			Eigen::Vector3d loc = abn->getTransform().translation();
			MakeBall(skel,abn->getCOM(),0.036,0.13);

			world->addSkeleton(skel);
		}
	
	}

}