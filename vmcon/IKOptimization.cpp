#include "IKOptimization.h"
#include <Eigen/Geometry>
#include <iostream>
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace Ipopt;
void 
IKOptimization::
AddTargetPositions(AnchorPoint ap,const Eigen::Vector3d& target)
{
	bool isAlreadyExist = false;
	for(int i =0;i<mTargets.size();i++)
	{
		if(mTargets[i].first.first ==ap.first){
			isAlreadyExist = true;
			mTargets[i].first.second = ap.second;
			mTargets[i].second = target;
		}
	}
	if(!isAlreadyExist)
		mTargets.push_back(std::make_pair(ap,target));
	// for(int i =0;i<mTargets.size();i++)
	// {
	// 	std::cout<<"Target "<<i<<" : "<<mTargets[i].first.first->getName()<<" -> "<<mTargets[i].second.transpose()<<std::endl;
	// }
}
const std::vector<std::pair<AnchorPoint,Eigen::Vector3d>>&
IKOptimization::
GetTargets()
{
	return mTargets;
}
IKOptimization::
IKOptimization(const SkeletonPtr& skeleton)
	:mSkeleton(skeleton),mSolution(skeleton->getPositions())
{	
	mSkeleton->computeForwardKinematics(true,false,false);
	mTargetOrientation[0] = mSkeleton->getBodyNode("HandL")->getTransform().rotation();
	mTargetOrientation[1] = mSkeleton->getBodyNode("HandR")->getTransform().rotation();
}
void
IKOptimization::
ClearTarget()
{
	mTargets.resize(0);
}
const Eigen::VectorXd&
IKOptimization::
GetSolution()
{
	return mSolution;
}
void
IKOptimization::
SetSolution(Eigen::VectorXd& sol)
{
	mSolution = sol;
}
IKOptimization::
~IKOptimization()
{

}

bool					
IKOptimization::
get_nlp_info(	Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
												Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
	n = mSkeleton->getNumDofs();
	m = 0;
	nnz_jac_g = 0;
	nnz_h_lag = n;
	index_style = TNLP::C_STYLE;
}


bool					
IKOptimization::
get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
												Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) 	
{
	for(int i =0;i<n;i++)
	{
		x_l[i] = mSkeleton->getDof(i)->getPositionLowerLimit();
		x_u[i] = mSkeleton->getDof(i)->getPositionUpperLimit();
	}
	return true;
}

bool					
IKOptimization::
get_starting_point(	Ipopt::Index n, bool init_x, Ipopt::Number* x,
													bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
													Ipopt::Index m, bool init_lambda,
													Ipopt::Number* lambda) 
{
	for(int i =0;i<n;i++)
		x[i] = mSkeleton->getDof(i)->getPosition();
	mSavePositions = mSkeleton->getPositions();
	return true;
}
Eigen::AngleAxisd GetDiff(const Eigen::Quaterniond& diff)
{
	Eigen::AngleAxisd diff1,diff2;
	diff1 = Eigen::AngleAxisd(diff);

	if(diff1.angle()>3.141592)
	{
		diff2.axis() = -diff1.axis();
		diff2.angle() = 3.141592*2 - diff1.angle();	
	}
	else
		diff2 = diff1;
	return diff2;
}
bool					
IKOptimization::
eval_f(	Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) 
{
	Eigen::VectorXd q(n);
	for(int i =0;i<n;i++)
		q[i] = x[i];

	mSkeleton->setPositions(q);
	mSkeleton->computeForwardKinematics(true,false,false);
	obj_value = 0;
	for(auto& target : mTargets)
	{
		obj_value += 0.5*(target.first.first->getTransform()*target.first.second - target.second).squaredNorm();
		// std::cout<<obj_value<<std::endl;
		Eigen::Quaterniond target_orientation;
		if(!target.first.first->getName().compare("HandL"))
			target_orientation = mTargetOrientation[0];
		else
			target_orientation = mTargetOrientation[1];
		Eigen::Quaterniond current_orientation(target.first.first->getTransform().rotation());
		Eigen::Quaterniond diff = target_orientation*current_orientation.inverse();

		// std::cout<<Eigen::AngleAxisd(diff).angle()<<std::endl;
		obj_value += 0.5*GetDiff(diff).angle();
	}
	return true;
}

bool					
IKOptimization::
eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) 
{
	Eigen::VectorXd q(n),g(n);
	for(int i =0;i<n;i++)
		q[i] = x[i];

	g.setZero();
	mSkeleton->setPositions(q);
	mSkeleton->computeForwardKinematics(true,false,false);
	for(auto& target: mTargets)
	{
		// dart::math::LinearJacobian J = mSkeleton->getLinearJacobian(target.first.first,target.first.second);
		dart::math::Jacobian J = mSkeleton->getWorldJacobian(target.first.first,target.first.second);
		// J.block(3,3,0,0) *= 100.0;
		J.block(6,3,0,0) *= 100.0;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix6d inv_singular_value;
		// Eigen::Matrix6d inv_singular_value;
		
		inv_singular_value.setZero();
		for(int k=0;k<6;k++)
		// for(int k=0;k<3;k++)
		{
			if(std::fabs(svd.singularValues()[k])<1e-6)
				inv_singular_value(k,k) = 0.0;
			else
				inv_singular_value(k,k) = 1.0/svd.singularValues()[k];
		}

		Eigen::MatrixXd J_inv = svd.matrixV()*inv_singular_value*svd.matrixU().transpose();

		Eigen::Vector3d x_minus_x_target = target.first.first->getTransform()*target.first.second - target.second;
		


		Eigen::Quaterniond target_orientation;
		if(!target.first.first->getName().compare("HandL"))
		{
			target_orientation = mTargetOrientation[0];
		}
		else
			target_orientation = mTargetOrientation[1];
		Eigen::Quaterniond current_orientation(target.first.first->getTransform().rotation());
		Eigen::Quaterniond diff = target_orientation*current_orientation.inverse();

		Eigen::Vector3d o_minus_o_target = GetDiff(diff).angle()*GetDiff(diff).axis();
		// std::cout<<GetDiff(diff).angle()<<std::endl;
		Eigen::Vector6d dir;
		dir.head(3) = -o_minus_o_target;
		dir.tail(3) = x_minus_x_target;
		// temp.setZero();
		// temp.head(3) = 0.1*o_minus_o_target;
		// temp.tail(3) = -x_minus_x_target;
		// g += J_inv*x_minus_x_target;
		g += J_inv*dir;

	}

	for(int i =0;i<n;i++)
		grad_f[i] = g[i];

	return true;
}

bool					
IKOptimization::
eval_g(	Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) 
{
	return true;
}

bool					
IKOptimization::
eval_jac_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x,
											Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
											Ipopt::Number* values) 
{
	return true;
}

bool					
IKOptimization::
eval_h( Ipopt::Index n, const Ipopt::Number* x, bool new_x,
										Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
										bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
										Ipopt::Index* jCol, Ipopt::Number* values) 
{
	int nnz = 0;

	if(values == NULL)
	{
		for(int i=0;i<n;i++)
		{
			iRow[nnz] = i;
			jCol[nnz++] = i;
		}
	}
	else
	{
		for(int i=0;i<n;i++){
			if(i<3)
				values[nnz++] = 100.0;
			else
				values[nnz++] = 1.0;
		}
	}

	return true;
}

void 					
IKOptimization::
finalize_solution(	Ipopt::SolverReturn status,
													Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
													Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
													Ipopt::Number obj_value,
													const Ipopt::IpoptData* ip_data,
													Ipopt::IpoptCalculatedQuantities* ip_cq) 
{
	for(int i=0;i<n;i++)
		mSolution[i] = x[i];
	mSkeleton->setPositions(mSavePositions);
	mSkeleton->computeForwardKinematics(true,false,false);
}
