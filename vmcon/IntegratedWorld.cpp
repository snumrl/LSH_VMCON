#include "IntegratedWorld.h"
#include "DART_helper.h"
#include "MusculoSkeletalSystem.h"
#include "Controller.h"
#include "Record.h"
#include "FSM/Machine.h"
#include <fstream>
#include <sstream>
#include <tinyxml.h>
#include <boost/filesystem.hpp>
 

using namespace FEM;
using namespace dart::dynamics;
using namespace dart::simulation;
IntegratedWorld::
IntegratedWorld()
{

}
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
		
#ifndef USE_JOINT_TORQUE
		mMusculoSkeletalSystem->TransformAttachmentPoints();
		mSoftWorld->TimeStepping(false);
#endif
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
	



#ifndef USE_JOINT_TORQUE
	mMusculoSkeletalSystem->ApplyForcesToSkeletons(mSoftWorld);
	if(need_fem_update)
	{
		mMusculoSkeletalSystem->TransformAttachmentPoints();
		mSoftWorld->TimeStepping();
	}
#else
	Eigen::VectorXd pd_forces = mMusculoSkeletalSystem->GetSkeleton()->getMassMatrix()*mController->mPDForces + mMusculoSkeletalSystem->GetSkeleton()->getCoriolisAndGravityForces();
	mMusculoSkeletalSystem->GetSkeleton()->setForces(pd_forces);
	if(need_fem_update)
	{
		mSoftWorld->SetTime(mSoftWorld->GetTime()+mSoftWorld->GetTimeStep());
	}
#endif
	mRigidWorld->step();

	mRecords.push_back(Record::Create());
	auto rec = mRecords.back();

	rec->Set(mRigidWorld,mSoftWorld,mMusculoSkeletalSystem,mController);
	
	std::string output_path("../output/");
	boost::filesystem::create_directories(output_path);
	
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
	MakeBalls(mRigidWorld,mMusculoSkeletalSystem,mBalls,5);

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
	// ReadRecord("../1210");
	
	// auto left_hand = mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandL");
	// auto right_hand = mMusculoSkeletalSystem->GetSkeleton()->getBodyNode("HandR");
	// for(int i=0;i<5;i++)
	// 	mBalls[i]->Release(mRigidWorld);

	// mBalls[1]->Attach(mRigidWorld,right_hand);
	// mBalls[4]->Attach(mRigidWorld,right_hand);
	// mController->GetMachine()->mJugglingInfo->SetCount(18);
	// std::cout<<mController->GetMachine()->mJugglingInfo->GetBallIndex()<<std::endl;
	// mController->GetMachine()->mPhase = 0;
	// mController->GetMachine()->mTimeElapsed = 100000;

	// mController->GetMachine()->mJugglingInfo->CountPlusPlus();
	// if(mBalls[mController->GetMachine()->mJugglingInfo->GetBallIndex()]->IsReleased())
	// {
	// 	// std::cout<<"Look ahead"<<mJugglingInfo->GetBallIndex()<<std::endl;
	// 	mController->GetMachine()->GenerateCatchMotions();
	// }
	// mController->GetMachine()->mJugglingInfo->CountMinusMinus();

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

	ofs<<"target "<<mController->GetTargetPositions().transpose()<<std::endl;
	ofs<<"time "<<mRigidWorld->getTime()<<std::endl;
	ofs<<"act "<<mMusculoSkeletalSystem->GetActivationLevels().transpose()<<std::endl;
	ofs.close();
}

void
IntegratedWorld::
ReadRecord(const std::string& path)
{
	std::ifstream ifs(path);
	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<path<<std::endl;
		return;
	}
	std::string str;
	std::string index;
	std::stringstream ss;
	std::shared_ptr<Record> new_record = Record::Create();
	while(!ifs.eof())
	{
		str.clear();
		index.clear();
		ss.clear();

		std::getline(ifs,str);
		ss.str(str);	
		ss>>index;

		Eigen::VectorXd eigen_vec;
		std::vector<double> vec;
		double val;
		if(!index.compare("soft"))
		{
			while(!ss.eof())
			{
				ss>>val;
				vec.push_back(val);
			}
			eigen_vec.resize(vec.size());
			for(int i=0;i<vec.size();i++)
			{
				eigen_vec[i] = vec[i];
			}
			new_record->soft_body_positions = eigen_vec;
		}
		else if(!index.compare("rpos"))
		{
			while(!ss.eof())
			{
				ss>>val;
				vec.push_back(val);
			}
			eigen_vec.resize(vec.size());
			for(int i=0;i<vec.size();i++)
			{
				eigen_vec[i] = vec[i];
			}
			new_record->rigid_body_positions.push_back(eigen_vec);
		}
		else if(!index.compare("rvel"))
		{
			while(!ss.eof())
			{
				ss>>val;
				vec.push_back(val);
			}
			eigen_vec.resize(vec.size());
			for(int i=0;i<vec.size();i++)
			{
				eigen_vec[i] = vec[i];
			}
			new_record->rigid_body_velocities.push_back(eigen_vec);

		}
		else if(!index.compare("act"))
		{
			while(!ss.eof())
			{
				ss>>val;
				vec.push_back(val);
			}
			eigen_vec.resize(vec.size());
			for(int i=0;i<vec.size();i++)
			{
				eigen_vec[i] = vec[i];
			}
			new_record->activation_levels = (eigen_vec);
		}
		else if(!index.compare("release"))
		{
			int rel;
			int count = 0;
			while(!ss.eof())
			{
				ss>>rel;
				if(rel==0)
					mBalls[count++]->Release(mRigidWorld);
				
			}
		}
		else if(!index.compare("time"))
		{
			ss>>val;
			new_record->t = val;
		}
		
	}
	ifs.close();

	mRigidWorld->setTime(new_record->t);
	for(int i =0;i<mRigidWorld->getNumSkeletons();i++)
		mRigidWorld->getSkeleton(i)->setPositions(new_record->rigid_body_positions[i]);
	for(int i =0;i<mRigidWorld->getNumSkeletons();i++)
		mRigidWorld->getSkeleton(i)->setVelocities(new_record->rigid_body_velocities[i]);

	mSoftWorld->SetTime(new_record->t);
	mSoftWorld->SetPositions(new_record->soft_body_positions);
	mMusculoSkeletalSystem->SetActivationLevels(new_record->activation_levels);


	// int juggling_count =1;
	// mController->mFSM->mJugglingFrame = juggling_count;
	// for(int i =0;i<juggling_count;i++){
	// 	mController->mFSM->Trigger("end");
	// 	mController->mFSM->Trigger("catch");
	// }
}