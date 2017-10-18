#ifndef __FEM_COROTATE_FEM_CST_H__
#define __FEM_COROTATE_FEM_CST_H__
#include "../Tensor3333.h"
namespace FEM
{


class CorotateFEMCst : public Cst
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	CorotateFEMCst(T k,T poisson_ratio,int i0,int i1,int i2,int i3,T volume,const Matrix3& invDms);
	void EvalPotentialEnergy(const VectorX& x) override;
	void EvalGradient(const VectorX& x) override;
	void EvalHessian(const VectorX& x) override;
	void GetPotentialEnergy(T& e) override;
	void GetGradient(VectorX& g) override;
	void GetHessian(std::vector<SMatrixTriplet>& h_triplets) override;
	void EvaluateDVector(const VectorX& x) override;
	void GetDVector(int& index,VectorX& d) override;
	void EvaluateJMatrix(int& index, std::vector<SMatrixTriplet>& J_triplets) override;
	void EvaluateLMatrix(std::vector<SMatrixTriplet>& L_triplets) override;
	int GetNumHessianTriplets() override;
	CstType GetType() override;
	void AddOffset(int offset) override;
protected:
	int mi0,mi1,mi2,mi3;
	T mVolume;
	T mMu,mLambda;
	Matrix3 mInvDm;

	//For parallization
	T 		mE;
	Vector12 mg;
	Matrix12 mH;
	Vector9 md;

	//For Cache
	Vector12 mX;
	Matrix3 mF;
	Matrix3 mR,mU,mV,mD;
	void ComputeF(const VectorX& x);
	void ComputeP(Matrix3& P);
	void ComputedPdF(Tensor3333& dPdF);
};
};
#endif