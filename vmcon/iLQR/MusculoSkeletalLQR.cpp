#include "MusculoSkeletalLQR.h"
#include "../MusculoSkeletalSystem.h"
#include "../MuscleOptimization.h"
#include "../IKOptimization.h"
#include "../Ball.h"
#include <boost/filesystem.hpp>

#include <tinyxml.h>

#include <fstream>
using namespace Ipopt;
using namespace dart::dynamics;
MusculoSkeletalLQR::
MusculoSkeletalLQR(
		const dart::simulation::WorldPtr& rigid_world,
		const std::shared_ptr<FEM::World>& soft_world,
		const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
		const std::vector<std::shared_ptr<Ball>>& balls,int max_iteration)
		:iLQR(max_iteration),
		mDofs(musculo_skeletal_system->GetSkeleton()->getNumDofs()),
		mSoftWorldDofs(soft_world->GetPositions().rows()),
		mRigidWorld(rigid_world),mSoftWorld(soft_world),mMusculoSkeletalSystem(musculo_skeletal_system),mBalls(balls),
		mTargetPositions(Eigen::VectorXd::Zero(musculo_skeletal_system->GetSkeleton()->getNumDofs())),
		mTargetVelocities(Eigen::VectorXd::Zero(musculo_skeletal_system->GetSkeleton()->getNumDofs())),
		mKp(Eigen::VectorXd::Constant(musculo_skeletal_system->GetSkeleton()->getNumDofs(),800.0)),
		mKv(Eigen::VectorXd::Constant(musculo_skeletal_system->GetSkeleton()->getNumDofs(),2*sqrt(800.0))),
		mInitCount(0)
{
	for(int i =0;i<6;i++)
	{
		mKp[mDofs-1-i] = 2.0*mKp[mDofs-1-i];	
		mKv[mDofs-1-i] = sqrt(2.0)*mKv[mDofs-1-i];
	}
	mMuscleOptimization = new MuscleOptimization(mSoftWorld,mRigidWorld,mMusculoSkeletalSystem);
	mMuscleOptimizationSolver = new IpoptApplication();
	
	mMuscleOptimizationSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	mMuscleOptimizationSolver->Options()->SetStringValue("jac_c_constant", "no");
	mMuscleOptimizationSolver->Options()->SetStringValue("hessian_constant", "yes");
	mMuscleOptimizationSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	mMuscleOptimizationSolver->Options()->SetIntegerValue("print_level", 2);
	mMuscleOptimizationSolver->Options()->SetIntegerValue("max_iter", 100);
	mMuscleOptimizationSolver->Options()->SetNumericValue("tol", 1e-4);

	mMuscleOptimizationSolver->Initialize();
	mMuscleOptimizationSolver->OptimizeTNLP(mMuscleOptimization);

	// mIKOptimization = new IKOptimization(mMusculoSkeletalSystem->GetSkeleton());

	// mIKSolver = new IpoptApplication();
	// mIKSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	// mIKSolver->Options()->SetStringValue("jac_c_constant", "yes");
	// mIKSolver->Options()->SetStringValue("hessian_constant", "yes");
	// mIKSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	// mIKSolver->Options()->SetIntegerValue("print_level", 2);
	// mIKSolver->Options()->SetIntegerValue("max_iter", 10);
	// mIKSolver->Options()->SetNumericValue("tol", 1e-4);

	// mIKSolver->Initialize();
	// mIKSolver->OptimizeTNLP(mIKOptimization);

	std::ifstream param("../vmcon/export/param.txt");
	param>>w_regularization>>w_smooth>>w_pos_track>>w_vel_track;
	param.close();
}
void
MusculoSkeletalLQR::
Initialze(
	const Eigen::Vector3d& pos_desired,
	const Eigen::Vector3d& vel_desired,
	int index,int next_index, BodyNode* next_body,bool next_ball_initially_attached,
	const std::vector<Eigen::VectorXd>& reference_motions,
	const Eigen::VectorXd& x0,const std::vector<Eigen::VectorXd>& u0)
{
	mBallIndex = index;
	mNextBallIndex = next_index;
	mNextBody = next_body;
	mNextBallInitiallyAttached = next_ball_initially_attached;
	std::cout<<"p_t : "<<pos_desired.transpose()<<std::endl;
	std::cout<<"v_t : "<<vel_desired.transpose()<<std::endl;
	mBallTargetPosition = pos_desired;
	mBallTargetVelocity = vel_desired;
	mReferenceMotions = reference_motions;
	Eigen::VectorXd u_lower(mMusculoSkeletalSystem->GetSkeleton()->getNumDofs());
	Eigen::VectorXd u_upper(mMusculoSkeletalSystem->GetSkeleton()->getNumDofs());
	for(int i =0;i<mMusculoSkeletalSystem->GetSkeleton()->getNumDofs();i++)
	{
		u_lower[i] = -0.5;//mMusculoSkeletalSystem->GetSkeleton()->getDof(i)->getPositionLowerLimit();
		u_upper[i] = 0.5;//mMusculoSkeletalSystem->GetSkeleton()->getDof(i)->getPositionUpperLimit();
	}
	// u_lower[mDofs] = 0.5;
	// u_upper[mDofs] = 2.0;
	// std::cout<<reference_motions.size()<<std::endl;
	mWritePath = "../output_lqr"+std::to_string((int)mInitCount++);
	boost::filesystem::create_directories(mWritePath);
	WriteXML(mWritePath+"/state.xml");

	Init(u0.size()+1,x0.rows()-mSoftWorldDofs,x0,u0,u_lower,u_upper);
}
void
MusculoSkeletalLQR::
ClipU(int i,double& lu,double& uu)
{
	// if(uu>mu_upper[i])
	// 	uu = mu_upper[i];
	// if(lu<mu_lower[i])
	// 	lu = mu_lower[i];
}
void
MusculoSkeletalLQR::
ClipX(int i,double& lx,double& ux)
{
	// int n = mMusculoSkeletalSystem->GetSkeleton()->getNumDofs();
	// if(i<n)
	// {
	// 	if(ux>mu_upper[i])
	// 		ux = mu_upper[i];
	// 	if(lx<mu_lower[i])
	// 		lx = mu_lower[i];
	// }
}

void
MusculoSkeletalLQR::
EvalCf(const Eigen::VectorXd& x,double& cf)
{
	SetState(x);

	Eigen::Vector3d ball_pos = mBalls[mBallIndex]->GetPosition();
	Eigen::Vector3d ball_vel = mBalls[mBallIndex]->GetVelocity();

	cf = 0.5*w_pos_track*(mBallTargetPosition - ball_pos).squaredNorm();
	cf += 0.5*w_vel_track*(mBallTargetVelocity - ball_vel).squaredNorm();
}

void
MusculoSkeletalLQR::
EvalC( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,double& c)
{
	c = 0.5*w_regularization*(u).squaredNorm();
	if(t==0)
		c+= 0.5*w_smooth*(u).squaredNorm();
	else
		c+= 0.5*w_smooth*(u-mu[t-1]).squaredNorm();
}

void
MusculoSkeletalLQR::
Evalf(  const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& f)
{
	SetState(x);
	if(t==0){
		if(mNextBallInitiallyAttached)
			mBalls[mNextBallIndex]->Attach(mRigidWorld,mNextBody);	
		else
			mBalls[mNextBallIndex]->Release(mRigidWorld);
	}
	SetControl(u,t);
	Step();
	GetState(f);
}

void
MusculoSkeletalLQR::
SetState(const Eigen::VectorXd& x,bool update_fem)
{
	mMusculoSkeletalSystem->GetSkeleton()->setPositions(x.head(mDofs));
	mMusculoSkeletalSystem->GetSkeleton()->setVelocities(x.block(mDofs,0,mDofs,1));

	mMusculoSkeletalSystem->GetSkeleton()->computeForwardKinematics(true,false,false);

	for(int i = 0;i<mBalls.size();i++)
	{
		mBalls[i]->GetSkeleton()->setPositions(x.block(2*mDofs+12*i+0,0,6,1));
		mBalls[i]->GetSkeleton()->setVelocities(x.block(2*mDofs+12*i+6,0,6,1));
		mBalls[i]->GetSkeleton()->computeForwardKinematics(true,false,false);	
	}

#ifndef USE_JOINT_TORQUE	
	mSoftWorld->SetPositions(x.tail(mSoftWorldDofs));
	if(update_fem)
	{
		mMusculoSkeletalSystem->TransformAttachmentPoints();
		mSoftWorld->TimeStepping(false);
	}
#endif
}
void
MusculoSkeletalLQR::
SetControl(const Eigen::VectorXd& u,double t)
{
	mTargetPositions = u+mReferenceMotions[t];
	if(t == 0){
		mTargetVelocities = (mReferenceMotions[t+1] - mReferenceMotions[t])/mSoftWorld->GetTimeStep();
	}
	else{
		mTargetVelocities = (mTargetPositions-(mReferenceMotions[t-1]+mu[t-1]))/mSoftWorld->GetTimeStep();
	}
}
void
MusculoSkeletalLQR::
GetState(Eigen::VectorXd& x)
{
	x.head(mDofs) = mMusculoSkeletalSystem->GetSkeleton()->getPositions();
	x.block(mDofs,0,mDofs,1) = mMusculoSkeletalSystem->GetSkeleton()->getVelocities();

	for(int i = 0;i<mBalls.size();i++)
	{
		x.block(2*mDofs+12*i+0,0,6,1) = mBalls[i]->GetSkeleton()->getPositions();
		x.block(2*mDofs+12*i+6,0,6,1) = mBalls[i]->GetSkeleton()->getVelocities();
	}

	x.tail(mSoftWorldDofs) = mSoftWorld->GetPositions();
}
void
MusculoSkeletalLQR::
Step()
{
	auto& skel = mMusculoSkeletalSystem->GetSkeleton();
	Eigen::VectorXd pos_diff = skel->getPositionDifferences(mTargetPositions,skel->getPositions());
	for(int i = 0;i<pos_diff.rows();i++)
		pos_diff[i] = dart::math::wrapToPi(pos_diff[i]);
	Eigen::VectorXd qdd_desired = pos_diff.cwiseProduct(mKp) + (mTargetVelocities - skel->getVelocities()).cwiseProduct(mKv);
	
	if(mBalls[mNextBallIndex]->IsReleased()) // check if already attached.
	{
		Eigen::Vector3d body_position = mNextBody->getTransform()*Eigen::Vector3d(0.0,0.02,0.03);
		Eigen::Vector3d ball_position = mBalls[mNextBallIndex]->GetPosition();
		if((body_position-ball_position).norm()<5E-2){
			mBalls[mNextBallIndex]->Attach(mRigidWorld,mNextBody);
		}
	}
#ifndef USE_JOINT_TORQUE
	static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->Update(qdd_desired);
	mMuscleOptimizationSolver->ReOptimizeTNLP(mMuscleOptimization);

	Eigen::VectorXd solution =  static_cast<MuscleOptimization*>(GetRawPtr(mMuscleOptimization))->GetSolution();

	mMusculoSkeletalSystem->SetActivationLevels(solution.tail(mMusculoSkeletalSystem->GetNumMuscles()));
	mMusculoSkeletalSystem->TransformAttachmentPoints();
	mSoftWorld->TimeStepping(false);
	double nn = mSoftWorld->GetTimeStep() / mRigidWorld->getTimeStep();
	for(int i =0; i<nn;i++)
	{
		mMusculoSkeletalSystem->ApplyForcesToSkeletons(mSoftWorld);
		mRigidWorld->step();
	}

#else
	double nn = mSoftWorld->GetTimeStep() / mRigidWorld->getTimeStep();

	for(int i =0; i<nn;i++)
	{
		Eigen::VectorXd torque = 	mMusculoSkeletalSystem->GetSkeleton()->getMassMatrix()*qdd_desired+
									mMusculoSkeletalSystem->GetSkeleton()->getCoriolisAndGravityForces();
		mMusculoSkeletalSystem->GetSkeleton()->setForces(torque);
		mRigidWorld->step();
	}
#endif
	}


void
MusculoSkeletalLQR::
Finalize(int& iteration)
{
	mWriteCount = 0;


	std::string path = mWritePath+"/iteration"+std::to_string(iteration);
	boost::filesystem::create_directories(path);
	mRigidWorld->setTime(0.0);
	mCost = 0;
	double c=0;
	for(int t = 0;t<mN-1;t++)
	{

		Evalf(mx[t],mu[t],t,mx[t+1]);
		EvalC(  mx[t],mu[t],t, c);
		WriteRecord(path);
	}
	double cf;
	EvalCf(mx[mN-1],cf);
	mCost += c;
	mCost +=cf;
	std::cout<<"Cost : "<<mCost<<"(cf : "<<cf<<")"<<std::endl;
	Eigen::VectorXd x_next = mx[mN-1];
	Eigen::VectorXd x_curr = mx[mN-1];
	auto abn =mBalls[mBallIndex]->GetConstraint()->getBodyNode2();
	mBalls[mBallIndex]->Release(mRigidWorld);
	while(true)
	{
		if(mRigidWorld->getTime()>1.0)
			break;
		SetState(x_curr);
		if(mBalls[mNextBallIndex]->IsReleased()) // check if already attached.
		{
			Eigen::Vector3d body_position = mNextBody->getTransform()*Eigen::Vector3d(0.0,0.02,0.03);
			Eigen::Vector3d ball_position = mBalls[mNextBallIndex]->GetPosition();
			if((body_position-ball_position).norm()<5E-2)
			{
				mBalls[mNextBallIndex]->Attach(mRigidWorld,mNextBody);
			}
		}
		mTargetPositions = (mReferenceMotions[mN-2]+mu[mN-2]);
		mTargetVelocities.setZero();
		Step();
		GetState(x_next);
		WriteRecord(path);
		x_curr = x_next;
	}
	
	SetState(mx[0]);
	SetControl(mu[0],0);
	mBalls[mBallIndex]->Attach(mRigidWorld,abn);
	mBalls[mNextBallIndex]->Release(mRigidWorld);
	Step();
}

void
MusculoSkeletalLQR::
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
MusculoSkeletalLQR::
WriteRecord(const std::string& path)
{
	std::string real_path = path +"/"+std::to_string(mWriteCount);
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
	ofs<<"target "<<mTargetPositions.transpose()<<std::endl;
	ofs<<"time "<<mRigidWorld->getTime()<<std::endl;
	ofs<<"act "<<mMusculoSkeletalSystem->GetActivationLevels().transpose()<<std::endl;
	ofs.close();
	mWriteCount++;
}






































































































void
MusculoSkeletalLQR::
EvalCfx(const Eigen::VectorXd& x,Eigen::VectorXd& cfx)
{
	cfx.setZero();

	Eigen::VectorXd x_i;
	double delta = 0.01;
	double cf_minus,cf_plus;
	double x_i_plus,x_i_minus;
	for(int i = 0;i<mSx_tilda;i++)
	{
		x_i = x;
		cf_minus =0;
		cf_plus =0;
		x_i_minus = x[i] - delta;
		x_i_plus = x[i] + delta;
	
		ClipX(i,x_i_minus,x_i_plus);
		
		x_i[i] = x_i_minus;
		EvalCf(x_i,cf_minus);
		x_i[i] = x_i_plus;
		EvalCf(x_i,cf_plus);

		cfx[i] = (cf_plus - cf_minus)/(x_i_plus - x_i_minus);
	}
}
void
MusculoSkeletalLQR::
EvalCfxx(const Eigen::VectorXd& x,Eigen::MatrixXd& cfxx)
{
	cfxx.resize(mSx_tilda,mSx_tilda);
	cfxx.setZero();
	Eigen::VectorXd x_i;
	double delta = 0.01;
	Eigen::VectorXd cfx_minus(mSx_tilda),cfx_plus(mSx_tilda);
	double x_i_minus,x_i_plus;
	for(int i = 0;i<mSx_tilda;i++)
	{
		x_i = x;
		cfx_minus.setZero();
		cfx_plus.setZero();
		x_i_minus = x[i] - delta;
		x_i_plus = x[i] + delta;

		ClipX(i,x_i_minus,x_i_plus);

		x_i[i] = x_i_minus;
		EvalCfx(x_i,cfx_minus);
		x_i[i] = x_i_plus;
		EvalCfx(x_i,cfx_plus);

		cfxx.col(i) = (cfx_plus - cfx_minus)/(x_i_plus - x_i_minus);
	}
}

void
MusculoSkeletalLQR::
EvalCx( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& cx)
{
	cx.setZero();

	Eigen::VectorXd x_i;
	double delta = 0.01;
	double c_minus,c_plus;
	double x_i_plus,x_i_minus;
	for(int i = 0;i<mSx_tilda;i++)
	{
		x_i = x;
		c_minus =0;
		c_plus =0;
		x_i_minus = x[i] - delta;
		x_i_plus = x[i] + delta;
	
		ClipX(i,x_i_minus,x_i_plus);
		
		x_i[i] = x_i_minus;
		EvalC(x_i,u,t,c_minus);
		x_i[i] = x_i_plus;
		EvalC(x_i,u,t,c_plus);

		cx[i] = (c_plus - c_minus)/(x_i_plus - x_i_minus);
	}
}
void
MusculoSkeletalLQR::
EvalCu( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& cu)
{
	cu.setZero();

	Eigen::VectorXd u_i;
	double delta = 0.01;
	double c_minus,c_plus;
	double u_i_plus,u_i_minus;
	for(int i = 0;i<mSu;i++)
	{
		u_i = u;
		c_minus =0;
		c_plus =0;
		u_i_minus = u[i] - delta;
		u_i_plus = u[i] + delta;
	
		ClipU(i,u_i_minus,u_i_plus);
		
		u_i[i] = u_i_minus;
		EvalC(x,u_i,t,c_minus);
		u_i[i] = u_i_plus;
		EvalC(x,u_i,t,c_plus);

		cu[i] = (c_plus - c_minus)/(u_i_plus - u_i_minus);
	}
}
void
MusculoSkeletalLQR::
EvalCxx(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cxx)
{
	cxx.resize(mSx_tilda,mSx_tilda);
	cxx.setZero();
	Eigen::VectorXd x_i;
	double delta = 0.01;
	Eigen::VectorXd cx_minus(mSx_tilda),cx_plus(mSx_tilda);
	double x_i_minus,x_i_plus;
	for(int i = 0;i<mSx_tilda;i++)
	{
		x_i = x;
		cx_minus.setZero();
		cx_plus.setZero();
		x_i_minus = x[i] - delta;
		x_i_plus = x[i] + delta;

		ClipX(i,x_i_minus,x_i_plus);

		x_i[i] = x_i_minus;
		EvalCx(x_i,u,t,cx_minus);
		x_i[i] = x_i_plus;
		EvalCx(x_i,u,t,cx_plus);

		cxx.col(i) = (cx_plus - cx_minus)/(x_i_plus - x_i_minus);
	}
}
void
MusculoSkeletalLQR::
EvalCxu(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cxu)
{
	cxu.resize(mSx_tilda,mSu);
	cxu.setZero();
	Eigen::VectorXd u_i;
	double delta = 0.01;
	Eigen::VectorXd cx_minus(mSx_tilda),cx_plus(mSx_tilda);
	double u_i_minus,u_i_plus;
	for(int i = 0;i<mSu;i++)
	{
		u_i = u;
		cx_minus.setZero();
		cx_plus.setZero();
		u_i_minus = u[i] - delta;
		u_i_plus = u[i] + delta;
	
		ClipU(i,u_i_minus,u_i_plus);

		u_i[i] = u_i_minus;
		EvalCx(x,u_i,t,cx_minus);
		u_i[i] = u_i_plus;
		EvalCx(x,u_i,t,cx_plus);

		cxu.col(i) = (cx_plus - cx_minus)/(u_i_plus - u_i_minus);
	}
}
void
MusculoSkeletalLQR::
EvalCuu(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cuu)
{
	cuu.resize(mSu,mSu);
	cuu.setZero();
	Eigen::VectorXd u_i;
	double delta = 0.01;
	Eigen::VectorXd cu_minus(mSu),cu_plus(mSu);
	double u_i_minus,u_i_plus;

	for(int i = 0;i<mSu;i++)
	{
		u_i = u;
		cu_minus.setZero();
		cu_plus.setZero();
		u_i_minus = u[i] - delta;
		u_i_plus = u[i] + delta;
	
		ClipU(i,u_i_minus,u_i_plus);

		u_i[i] = u_i_minus;
		EvalCu(x,u_i,t,cu_minus);
		u_i[i] = u_i_plus;
		EvalCu(x,u_i,t,cu_plus);

		cuu.col(i) = (cu_plus - cu_minus)/(u_i_plus - u_i_minus);
	}
}



void
MusculoSkeletalLQR::
Evalfx( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& fx)
{
	fx.resize(mSx_tilda,mSx_tilda);
	fx.setZero();

	Eigen::VectorXd x_i;
	double delta = 0.01;
	Eigen::VectorXd fx_i_minus(mSx),fx_i_plus(mSx);
	double x_i_minus,x_i_plus;

	
	double nn = mSoftWorld->GetTimeStep() / mRigidWorld->getTimeStep();
	for(int i = 0;i<mSx_tilda;i++)
	{
		if(i<2*mDofs)//||(i>=2*mDofs+6*mBallIndex&&i<2*mDofs+6*mBallIndex+6 ))
		{
			x_i = x;

			fx_i_minus.setZero();
			fx_i_plus.setZero();

			x_i_minus = x[i] - delta;
			x_i_plus = x[i] + delta;

			ClipX(i,x_i_minus,x_i_plus);

			x_i[i] = x_i_minus;
			// Evalf(x_i,u,t,fx_i_minus);
			SetState(x_i,false);
			for(int i =0; i<nn;i++)
				mRigidWorld->step();
			GetState(fx_i_minus);
			x_i[i] = x_i_plus;
			// Evalf(x_i,u,t,fx_i_plus);
			SetState(x_i,false);
			for(int i =0; i<nn;i++)
				mRigidWorld->step();
			GetState(fx_i_plus);
			fx.col(i) = (fx_i_plus.block(0,0,mSx_tilda,1) - fx_i_minus.block(0,0,mSx_tilda,1))/(x_i_plus - x_i_minus);
		}
		else
			fx.col(i).setZero();
	}
}
void
MusculoSkeletalLQR::
Evalfu( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& fu)
{
	fu.resize(mSx_tilda,mSu);
	fu.setZero();

	Eigen::VectorXd u_i;
	double delta = 0.01;
	Eigen::VectorXd fu_i_minus(mSx),fu_i_plus(mSx);
	double u_i_minus,u_i_plus;
	for(int i = 0;i<mSu;i++)
	{
		u_i = u;

		fu_i_minus.setZero();
		fu_i_plus.setZero();

		u_i_minus = u[i] - delta;
		u_i_plus = u[i] + delta;

		ClipU(i,u_i_minus,u_i_plus);

		u_i[i] = u_i_minus;
		Evalf(x,u_i,t,fu_i_minus);
		u_i[i] = u_i_plus;
		Evalf(x,u_i,t,fu_i_plus);

		fu.col(i) = (fu_i_plus.block(0,0,mSx_tilda,1) - fu_i_minus.block(0,0,mSx_tilda,1))/(u_i_plus - u_i_minus);
	}
}
