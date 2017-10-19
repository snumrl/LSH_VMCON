#include "World.h"
using namespace FEM;

World::
World(	IntegrationMethod im,
		T time_step,
		int max_iteration,
		Vector3 gravity,
		T damping_coeff)
	:mIntegrationMethod(im),
	mTimeStep(time_step),
	mMaxIteration(max_iteration),
	mGravity(gravity)
	mDampingCoefficient(damping_coeff),
	mNumVertices(0),
	mConstraintDofs(0),
	mIsInitialized(false)
{

}

void
World::
Initialize()
{

}
void
World::
TimeStepping()
{
	if(!mIsInitialized)
	{
		std::cout<<"Engine is not initialized."<<std::endl;
		return;
	}
	VectorX x_next(mNumVertices*3);
	mExternalForces.setZero();
	for(int i=0;i<mNumVertices;i++)
		mExternalForces.block(i) = mGravity;

	mq = mPositions + mTimeStep*mVelocities + (mTimeStep*mTimeStep)*(mInvMassMatrix*mExternalForces);

	switch(mIntegrationMethod)
	{
	case NEWTON_METHOD:
	break;
		x_next = IntegrateNewtonMethod();
	case QUASI_STATIC:
	break;
		x_next = IntegrateQuasiStatic();
	case PROJECTIVE_DYNAMICS:
	break;
		x_next = IntegrateProjectiveDynamics();
	case PROJECTIVE_QUASI_STATIC:
		x_next = IntegrateProjectiveQuasiStatic();
	break;
	default:
	return;
	};
	IntegratePositionsAndVelocities(x_next);

	mTime += mTimeStep;
}
void
World::
AddBody(const VectorX& x0, const,const std::vector<std::shared_ptr<Cst>>& constraints,T m)
{
	if(mIsInitialized)
	{
		std::cout<<"Add Body before initializing engine."<<std::endl;
		return;
	}
	int nv = x0.rows()/3;
	mNumVertices += nv;

	VectorX prev_x(mPositions);
	mPositions.resize(mNumVertices*3);
	mPositions.head(prev_x.rows()) = prev_x;
	mPositions.tail(x0.rows()) = x0;

	mConstraints.insert(mConstraints.end(),constraints.begin(),constraints.end());
	T unit_mass = m/((T)nv);
	for(int i=0;i<nv;i++)
		mUnitMass.push_back(unit_mass);
}
void
World::
AddConstraint(std::shared_ptr<Cst> c)
{
	mConstraints.push_back(c):
	if((mIntegrationMethod == IntegrationMethod::PROJECTIVE_DYNAMICS||
		mIntegrationMethod == IntegrationMethod::PROJECTIVE_QUASI_STATIC)&&
		mIsInitialized)
	{
		mConstraintDofs = 0;
		Precompute();
	}
}
void
World::
RemoveConstraint(std::shared_ptr<Cst> c)
{
	bool isRemoved = false;
	for(int i =0;i<mConstraints.size();i++)
	{
		if(mConstraints[i]==c)
		{
			mConstraints.erase(mConstraints.begin()+i);
			isRemoved = true;
			break;
		}
	}

	if((mIntegrationMethod == IntegrationMethod::PROJECTIVE_DYNAMICS||
		mIntegrationMethod == IntegrationMethod::PROJECTIVE_QUASI_STATIC)&&
		mIsInitialized)
	{
		Precompute();
	}
}
VectorX
World::
IntegrateNewtonMethod()
{
	VectorX x_next(mNumVertices*3);

	x_next = mq;

	VectorX g_k(mNumVertices*3);
	SMatrix H_k(mNumVertices*3,mNumVertices*3);
	for(int k=0;k<mMaxIteration;k++)
	{
		EvaluateGradient(x_next,g_k);

		if(g_k.squaredNorm()<BIG_EPSILON)
			break;

		EvaluateHessian(x_next,H_k);
		FactorizeLDLT(H_k,mSolver);
		VectorX d = -mSolver.solve(g_k);

		T alpha = ComputeStepSize(x_next,g_k,d);

		x_next += alpha*d;
	}
	InversionFree(x_next);
	return x_next;
}
VectorX
World::
IntegrateQuasiStatic()
{
	VectorX x_next(mNumVertices*3);

	x_next = mq;

	VectorX g_k(mNumVertices*3);
	SMatrix H_k(mNumVertices*3,mNumVertices*3);
	for(int k=0;k<mMaxIteration;k++)
	{
		EvaluateConstraintsGradient(x_next,g_k);

		if(g_k.squaredNorm()<BIG_EPSILON)
			break;

		EvaluateConstraintsHessian(x_next,H_k);
		FactorizeLDLT(H_k,mSolver);
		VectorX d = -mSolver.solve(g_k);

		T alpha = ComputeStepSize(x_next,g_k,d);

		x_next += alpha*d;
	}
	InversionFree(x_next);
	return x_next;
}
VectorX
World::
IntegrateProjectiveDynamics()
{
	VectorX x_next(mNumVertices*3);
	VectorX b(mNumVertices*3);
	VectorX d(mConstraintDofs*3);

	b = (1.0/(mTimeStep*mTimeStep))*mMassMatrix*mq;

	x_next = mq;

	for(int k=0;k<mMaxIteration;k++)
	{
		EvaluateDVector(x_next,d);
		x_next = mSolver.solve(b+mJ*d);
	}
	InversionFree(x_next);
	return x_next;
}
VectorX
World::
IntegrateProjectiveQuasiStatic()
{
	VectorX x_next(mNumVertices*3);
	VectorX d(mConstraintDofs*3);

	x_next = mPositions;

	for(int k=0;k<mMaxIteration;k++)
	{
		EvaluateDVector(x_next,d);
		x_next = mSolver.solve(mJ*d);
	}
	InversionFree(x_next);
	return x_next;
}
void
World::
FactorizeLLT(const SMatrix& A, Eigen::SimplicialLLT<SMatrix>& llt_solver)
{
	Eigen::SparseMatrix<double> A_prime = A;
	lltSolver.analyzePattern(A_prime);
	lltSolver.factorize(A_prime);
	double damping = 1E-6;
	bool success = true;
	while (lltSolver.info() != Eigen::Success)
	{
		damping *= 10;
		A_prime = A + damping*mIdentityMatrix;
		lltSolver.factorize(A_prime);
		success = false;
	}
	if (!success)
		std::cout << "factorize failure (damping : " << damping<<" )"<<std::endl;
}
void
World::
FactorizeLDLT(const SMatrix& A, Eigen::SimplicialLDLT<SMatrix>& ldlt_solver)
{
	Eigen::SparseMatrix<double> A_prime = A;
	ldltSolver.analyzePattern(A_prime);
	ldltSolver.factorize(A_prime);
	double damping = 1E-6;
	bool success = true;
	while (ldltSolver.info() != Eigen::Success)
	{
		damping *= 10;
		A_prime = A + damping*mIdentityMatrix;
		ldltSolver.factorize(A_prime);
		success = false;
	}
	if (!success)
		std::cout << "factorize failure (damping : " << damping<<" )"<<std::endl;
}
T
World::
EvaluateEnergy(const VectorX& x)
{
	VectorX x_q = x - mq;

	T val = EvaluateConstraintsEnergy(x);

	if(mIntegrationMethod == NEWTON_METHOD)
		val += 0.5*(1.0/(mTimeStep*mTimeStep))*(x_q.dot(mMassMatrix*x_q));

	return val;
}
void
World::
EvaluateGradient(const VectorX& x,VectorX& g)
{
	EvaluateConstraintsGradient(x,g);
	
	if(mIntegrationMethod == NEWTON_METHOD)
		g += (1.0/(mTimeStep*mTimeStep))*mMassMatrix*(x - mq);
}
void
World::
EvaluateHessian(const VectorX& x,SMatrix& H)
{
	EvaluateConstraintsHessian(x,H);

	if(mIntegrationMethod == NEWTON_METHOD)
		H = H + (1.0/(mTimeStep*mTimeStep))*mMassMatrix;
}

T
World::
EvaluateConstraintsEnergy(const VectorX& x)
{
	T energy=0;
	
#pragma omp for
	for(int i=0;i<mConstraints.size();i++)
	{
		mConstraints[i]->EvaluatePotentialEnergy(x);
	}

	for(auto& c : mConstraints)
	{
		c->GetPotentialEnergy(energy);
	}


	return energy;
}
void
World::
EvaluateConstraintsGradient(const VectorX& x,VectorX& g)
{
	g.resize(mNumVertices*3);
	g.setZero();

#pragma omp for
	for(int i=0;i<mConstraints.size();i++)
	{
		mConstraints[i]->EvaluateGradient(x);
	}

	for(auto& c : mConstraints)
	{
		c->GetGradient(g);
	}

}
void
World::
EvaluateConstraintsHessian(const VectorX& x,SMatrix& H)
{
	H.resize(mNumVertices*3,mNumVertices*3);
	std::vector<SMatrixTriplet> h_triplets;

#pragma omp for
	for(int i=0;i<mConstraints.size();i++)
	{
		mConstraints[i]->EvaluateHessian(x);
	}

	for(auto& c : mConstraints)
	{
		c->GetHessian(h_triplets);
	}

	H.setFromTriplets(h_triplets.cbegin(),h_triplets.cend());
}
void
World::
ComputeStepSize(const VectorX& x, const VectorX& g,const VectorX& d)
{
	T alpha = 1.0;
	T c1 = 0.03;
	T c2 = 0.5;
	T f_x,f_x_next;
	VectorX x_next;

	f_x = EvaluateEnergy(x);

	T g_dot_d = g.dot(d);

	for(i=0;i<16;i++)
	{
		x_next = x + alpha*d;

		f_x_next = EvaluateEnergy(x_next);

		if(f_x + alpha*c1*g_dot_d>=f_x_next)
			break;

		alpha *= c2;
	}

	return alpha;
}
void
World::
EvaluateDVector(const VectorX& x,VectorX& d)
{
	d.resize(mConstraintDofs*3);

#pragma omp for
	for(int i=0;i<mConstraints.size();i++)
	{
		mConstraints[i]->EvaluateDVector(x);
	}

	int index = 0;
	for(auto& c : mConstraints)
	{
		c->GetDVector(index,d);
	}
}
void
World::
EvaluateJMatrix(SMatrix& J)
{
	J.resize(mNumVertices*3,mConstraintDofs*3);
	std::vector<SMatrixTriplet> J_triplets;

	int index = 0;
	for(auto& c : mConstraints)
	{
		c->EvaluateJMatrix(index,J_triplets);
	}

	J.setFromTriplets(J_triplets.cbegin(),J_triplets.cend());
}
void
World::
EvaluateLMatrix(SMatrix& L)
{
	L.resize(mNumVertices*3,mNumVertices*3);
	std::vector<SMatrixTriplet> L_triplets;

	for(auto& c: mConstraints)
	{
		c->EvaluateLMatrix(L_triplets);
	}

	L.setFromTriplets(L_triplets.cbegin(),L_triplets.cend());
}
void
World::
Precompute()
{
	mConstraintDofs = 0;

	//For computing constraint's dofs
	Eigen::VectorX d_temp(3*mConstraints.size()*3);
	int index = 0;
	for(auto& c : mConstraints)
	{
		c->GetDVector(index,d_temp);
	}
	mConstraintDofs = index;

	EvaluateLMatrix(mL);
	EvaluateJMatrix(mJ);

	SMatrix H2ML = (1.0/(mTimeStep*mTimeStep))*mMassMatrix+mL;
	if(mIntegrationMethod == PROJECTIVE_DYNAMICS)
		FactorizeLDLT(H2ML,mSolver);
	else if(mIntegrationMethod == PROJECTIVE_QUASI_STATIC)
		FactorizeLDLT(mL,mSolver);
}
void
World::
InversionFree(VectorX& x)
{
	//TODO
}
void
World::
IntegratePositionsAndVelocities(const VectorX& x_next)
{
	mVelocities = (1.0/mTimeStep)*(x_next - mPositions);
	mPositions = x_next;
}