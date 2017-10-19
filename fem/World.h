#ifndef __FEM_WORLD_H__
#define __FEM_WORLD_H__
#include "Constraint/Constraint.h"
#include "global_headers.h"
namespace FEM
{
enum IntegrationMethod
{
	NEWTON_METHOD,
	QUASI_STATIC,
	PROJECTIVE_DYNAMICS,
	PROJECTIVE_QUASI_STATIC
};
class World
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	World(	IntegrationMethod im = NEWTON_METHOD,
			T time_step = 1.0/120.0,
			int max_iteration = 100,
			Vector3 gravity = Vector3(0,-9.81,0),
			T damping_coeff = 0.999);

	void Initialize();
	void TimeStepping();

	void AddBody(const VectorX& x0, const,const std::vector<std::shared_ptr<Cst>>& constraints, T m = 1.0);
	void AddConstraint(std::shared_ptr<Cst> c);
	void RemoveConstraint(std::shared_ptr<Cst> c);

private:
	VectorX IntegrateNewtonMethod();
	VectorX IntegrateQuasiStatic();
	VectorX IntegrateProjectiveDynamics();
	VectorX IntegrateProjectiveQuasiStatic();

	void FactorizeLLT(const SMatrix& A, Eigen::SimplicialLLT<SMatrix>& llt_solver);
	void FactorizeLDLT(const SMatrix& A, Eigen::SimplicialLDLT<SMatrix>& ldlt_solver);

	//For Newton method, Quasi-static
	T EvaluateEnergy(const VectorX& x);
	void EvaluateGradient(const VectorX& x,VectorX& g);
	void EvaluateHessian(const VectorX& x,SMatrix& H);
	
	T EvaluateConstraintsEnergy(const VectorX& x);
	void EvaluateConstraintsGradient(const VectorX& x,VectorX& g);
	void EvaluateConstraintsHessian(const VectorX& x,SMatrix& H);
	
	void ComputeStepSize(const VectorX& x, const VectorX& g,const VectorX& d);
	//For Projective Dynamics, Projective Quasi-static
	void EvaluateDVector(const VectorX& x,VectorX& d);
	void EvaluateJMatrix(SMatrix& J);
	void EvaluateLMatrix(SMatrix& L);

	void Precompute();
	//For detailing
	void InversionFree(VectorX& x);

	void IntegratePositionsAndVelocities(const VectorX& x_next);

private:
	bool isInitialized;
	int mNumVertices;
	int mConstraintDofs;
	int mMaxIteration;

	T mTimeStep,mTime;
	T mDampingCoefficient;
	Vector3 mGravity;

	IntegrationMethod mIntegrationMethod;

	std::vector<T> mUnitMass;
	std::vector<std::shared_ptr<Cst>> mConstraints;

	VectorX mPositions,mVelocities;
	VectorX mExternalForces;

	SMatrix mMassMatrix,mInvMassMatrix,mIdentityMatrix;
	
	//For Projective Dynamics, Projective Quasi-static
	VectorX mq;
	SMatrix mJ,mL;

	Eigen::SimplicialLDLT<SMatrix> mSolver;
};
};

#endif