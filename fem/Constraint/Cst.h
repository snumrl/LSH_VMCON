#ifndef __FEM_CST_H__
#define __FEM_CST_H__
#include "../global_headers.h"
#include "Cst_Data.h"
namespace FEM
{
enum CstType
{
	ATTACHMENT,
	COROTATE_FEM,
	LINEAR_MUSCLE
};
class Cst
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Cst(T k):mStiffness(k){};
	virtual void EvaluatePotentialEnergy(const VectorX& x) = 0;
	virtual void EvaluateGradient(const VectorX& x) = 0;
	virtual void EvaluateHessian(const VectorX& x) = 0;

	virtual void GetPotentialEnergy(T& e) = 0;
	virtual void GetGradient(VectorX& g) = 0;
	virtual void GetHessian(std::vector<SMatrixTriplet>& h_triplets) = 0;

	virtual void EvaluateDVector(const VectorX& x) = 0;
	virtual void GetDVector(int& index,VectorX& d) = 0;
	virtual void EvaluateJMatrix(int& index, std::vector<SMatrixTriplet>& J_triplets) = 0;
	virtual void EvaluateLMatrix(std::vector<SMatrixTriplet>& L_triplets) = 0;

	virtual int GetNumHessianTriplets() = 0;
	virtual CstType GetType() = 0;
	virtual void AddOffset(int offset) = 0;
protected:
	T mStiffness;

};

};
#endif