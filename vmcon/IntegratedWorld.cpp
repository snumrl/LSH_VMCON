#include "IntegratedWorld.h"
#include "DART_helper.h"
#include "MusculoSkeletalSystem.h"
#include "Controller.h"
#include "Record.h"
#include <fstream>
#include <sstream>
#include <tinyxml.h>
using namespace FEM;
using namespace dart::dynamics;
using namespace dart::simulation;
// #define USE_LS_MUSCLE
IntegratedWorld::
IntegratedWorld()
{

}
extern int v_target_from_argv;
bool
IntegratedWorld::
TimeStepping()
{
	bool need_fem_update = false;

	// std::cout<<"timestepping"<<std::endl;
	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getPositions().transpose()<<std::endl;

	if(mSoftWorld->GetTime()<=mRigidWorld->getTime())
	{
		need_fem_update = true;
		// std::cout<<"mController step"<<std::endl;
		mController->Step();
	}

	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getPositions().transpose()<<std::endl;
	// std::cout<<std::endl;
	// auto pd_forces = mController->ComputePDForces();
	// mMusculoSkeletalSystem->GetSkeleton()->clearConstraintImpulses();
	// mMusculoSkeletalSystem->GetSkeleton()->clearInternalForces();


	// std::cout<<mBalls[0]->GetPosition().transpose()<<std::endl;
	// auto interesting = mRigidWorld->getConstraintSolver()->mManualConstraints[0];
	// std::cout<<((dart::constraint::WeldJointConstraint*)interesting.get())->mJacobian2<<std::endl;
	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getPositions()[0]<<" ";
	// std::cout<<mBalls[0]->GetVelocity().transpose()<<std::endl;
			// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getExternalForces().transpose()<<std::endl;
	// std::cout<<mMusculoSkeletalSystem->GetSkeleton()->getConstraintForces().transpose()<<std::endl;
	// std::cout<<pd_forces.transpose()<<std::endl;


	// Eigen::VectorXd pd_forces = mMusculoSkeletalSystem->GetSkeleton()->getMassMatrix()*mController->mPDForces + mMusculoSkeletalSystem->GetSkeleton()->getCoriolisAndGravityForces();
	// mMusculoSkeletalSystem->GetSkeleton()->setForces(pd_forces);



	mMusculoSkeletalSystem->ApplyForcesToSkeletons(mSoftWorld);
	if(need_fem_update)
	{
		mMusculoSkeletalSystem->TransformAttachmentPoints();
#ifndef USE_LS_MUSCLE
		mSoftWorld->TimeStepping();
#else
		mSoftWorld->SetTime(mSoftWorld->GetTime()+mSoftWorld->GetTimeStep());
#endif
	}

	mRigidWorld->step();

	mRecords.push_back(Record::Create());
	auto rec = mRecords.back();

	rec->Set(mRigidWorld,mSoftWorld,mMusculoSkeletalSystem,mController);
	

	std::string output_path("../output_"+std::to_string(v_target_from_argv)+"/");
	if(mRigidWorld->getTime()>3.0)
		exit(0);
	WriteRecord(output_path);


	return need_fem_update;
}
std::shared_ptr<IntegratedWorld>
IntegratedWorld::
Clone()
{
	auto new_iw = Create();

	new_iw->mSoftWorld = mSoftWorld->Clone();
	new_iw->mRigidWorld = mRigidWorld->clone();
	new_iw->mMusculoSkeletalSystem = mMusculoSkeletalSystem->Clone(mSoftWorld,mRigidWorld);
	
	return new_iw;
}
std::shared_ptr<IntegratedWorld>
IntegratedWorld::
Create()
{
	auto iw = new IntegratedWorld();
	return std::shared_ptr<IntegratedWorld>(iw);
}

void
IntegratedWorld::
Initialize()
{
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
	// mRigidWorld->setGravity(Eigen::Vector3d(0,0,0));
	mMusculoSkeletalSystem = MusculoSkeletalSystem::Create();
	MakeSkeleton(mMusculoSkeletalSystem);
	MakeMuscles("../vmcon/export/muscle_params.xml",mMusculoSkeletalSystem);

	mMusculoSkeletalSystem->Initialize(mSoftWorld,mRigidWorld);
	mSoftWorld->Initialize();

	double ball_mass[10] = {0.13,0.5,1.0,3.0,5.0,10.0,20.0};
	
	for(int i =0;i<1;i++)
	{
		SkeletonPtr skel = Skeleton::create("ball_"+std::to_string(i));

		bool is_left_hand = i%2;
		if(is_left_hand)
		{
			auto* abn =mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandL");
			Eigen::Vector3d loc = abn->getTransform().translation();
			Eigen::Vector3d bp = abn->getCOM();
			bp[1] +=0.1;
			// bp[0] +=0.1;
			// MakeBall(skel,bp,0.036,ball_mass[v_target_from_argv]);
			MakeBall(skel,bp,0.036,0.13);

			// mBalls.push_back(std::make_shared<Ball>(std::make_shared<dart::constraint::WeldJointConstraint>(skel->getBodyNode(0),abn),skel));
			mBalls.push_back(std::make_shared<Ball>(nullptr,skel));
			mRigidWorld->addSkeleton(skel);
			mBalls.back()->Attach(mRigidWorld,abn);
		}
		else
		{
			auto* abn =mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR");
			Eigen::Vector3d loc = abn->getTransform().translation();
			Eigen::Vector3d bp = abn->getCOM();
			// bp[1] -=0.02;
			bp[0] +=0.02;
			bp[2] +=0.03;
			// MakeBall(skel,bp,0.036,ball_mass[v_target_from_argv]);
			MakeBall(skel,bp,0.036,0.13);
			// mBalls.push_back(std::make_shared<Ball>(std::make_shared<dart::constraint::WeldJointConstraint>(skel->getBodyNode(0),abn),skel));
			mBalls.push_back(std::make_shared<Ball>(nullptr,skel));
			mRigidWorld->addSkeleton(skel);
			// mBalls.back()->Attach(mRigidWorld,abn);	
			mBalls.back()->Release(mRigidWorld);	
		}
	
	}

	// for(int i=0;i<1;i++)
	// {
		
	// 	BodyNode* abn = mMusculoSkeletalSystem->GetSkeleton()->getBodyNode((i%2==0?"HandL":"HandR"));
	// 	MakeBall(skel,abn->getCOM(),0.036,0.13);
	// 	mBalls.push_back(std::make_shared<Ball>(skel));
	// 	mBalls.back()->Attach(mRigidWorld,abn);
	// 	mRigidWorld->addSkeleton(skel);
	// }
	WriteXML("../output/world_state.xml");
	mController = Controller::Create(mSoftWorld,mRigidWorld,mMusculoSkeletalSystem,mBalls);

}

void
IntegratedWorld::
SetRecord(int& frame)
{
	if(frame < 0)
		frame = 0;
	if(frame>=mRecords.size())
		frame=0;
	mRecords[frame]->Get(mRigidWorld,mSoftWorld,mMusculoSkeletalSystem,mController);

}

void
IntegratedWorld::
WriteXML(const std::string& path)
{
	TiXmlDocument doc;

    Eigen::VectorXd pos = mSoftWorld->GetPositions();
    std::vector<TiXmlElement*> node_elems;
    for(int i=0;i<pos.rows()/3;i++)
    {
		node_elems.push_back(new TiXmlElement("node"));

		node_elems.back()->SetDoubleAttribute("x",pos[3*i+0]);
		node_elems.back()->SetDoubleAttribute("y",pos[3*i+1]);
		node_elems.back()->SetDoubleAttribute("z",pos[3*i+2]);
    }

    auto& cons = mSoftWorld->GetConstraints();
    std::vector<TiXmlElement*> c_elems;
    for(auto& c : cons)
    {
		c_elems.push_back(new TiXmlElement("constraint"));
    	if(dynamic_cast<FEM::AttachmentCst*>(c.get()) != nullptr)
		{
			FEM::AttachmentCst* ac = dynamic_cast<FEM::AttachmentCst*>(c.get());	
			int i0 = ac->GetI0();
			c_elems.back()->SetAttribute("type","attachment");
			c_elems.back()->SetAttribute("i0",i0);
		}
		else if(dynamic_cast<FEM::CorotateFEMCst*>(c.get()) != nullptr)
		{
			FEM::CorotateFEMCst* cc = dynamic_cast<FEM::CorotateFEMCst*>(c.get());
			int i0 = cc->GetI0();
			int i1 = cc->GetI1();
			int i2 = cc->GetI2();
			int i3 = cc->GetI3();
			c_elems.back()->SetAttribute("type","corotate");
			c_elems.back()->SetAttribute("i0",i0);
			c_elems.back()->SetAttribute("i1",i1);
			c_elems.back()->SetAttribute("i2",i2);
			c_elems.back()->SetAttribute("i3",i3);

		}
		else if(dynamic_cast<FEM::LinearMuscleCst*>(c.get()) != nullptr)
		{
			FEM::LinearMuscleCst* cc = dynamic_cast<FEM::LinearMuscleCst*>(c.get());
			int i0 = cc->GetI0();
			int i1 = cc->GetI1();
			int i2 = cc->GetI2();
			int i3 = cc->GetI3();
			Eigen::Vector3d dir = cc->GetFiberDirection();
			c_elems.back()->SetAttribute("type","muscle");
			c_elems.back()->SetAttribute("i0",i0);
			c_elems.back()->SetAttribute("i1",i1);
			c_elems.back()->SetAttribute("i2",i2);
			c_elems.back()->SetAttribute("i3",i3);
			c_elems.back()->SetDoubleAttribute("dx",dir[0]);
			c_elems.back()->SetDoubleAttribute("dy",dir[1]);
			c_elems.back()->SetDoubleAttribute("dz",dir[2]);
		}

    }

    TiXmlElement* soft_elem = new TiXmlElement( "Soft");
    TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );  

    TiXmlElement* nodes_elem = new TiXmlElement( "Nodes" );
    for(auto& elem: node_elems)
    {
    	nodes_elem->LinkEndChild(elem);
    }
    TiXmlElement* cons_elem = new TiXmlElement( "Constraints" );
    for(auto& elem : c_elems)
    {
    	cons_elem->LinkEndChild(elem);
    }


	doc.LinkEndChild( decl ); 
	soft_elem->LinkEndChild(nodes_elem);
	soft_elem->LinkEndChild(cons_elem);
	doc.LinkEndChild( soft_elem );
    // doc.LinkEndChild(nodes_elem);
    // doc.LinkEndChild(cons_elem);
    doc.SaveFile( path.c_str() );

}
void
IntegratedWorld::
WriteRecord(const std::string& path)
{
	std::string real_path = path + std::to_string(mRecords.size()-1);
	std::ofstream ofs(real_path);
	Eigen::VectorXd soft_pos = mSoftWorld->GetPositions();
	ofs<<"soft "<<soft_pos.transpose()<<std::endl;

	for(int i =0;i<mRigidWorld->getNumSkeletons();i++)
	{
		ofs<<"rpos ";
		ofs<<mRigidWorld->getSkeleton(i)->getPositions().transpose()<<std::endl;
	}

	for(int i =0;i<mRigidWorld->getNumSkeletons();i++)
	{
		ofs<<"rvel ";
		ofs<<mRigidWorld->getSkeleton(i)->getVelocities().transpose()<<std::endl;
	}
	ofs<<"time "<<mRigidWorld->getTime();
	ofs.close();
}
