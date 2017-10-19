#ifndef __FEM_ATTACHMENT_CST_H__
#define __FEM_ATTACHMENT_CST_H__
#include "Cst.h"

namespace FEM
{
class AttachmentCst : public Cst
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	AttachmentCst(T k,int i0,const Vector3& p);
	void EvaluatePotentialEnergy(const VectorX& x) override;
	void EvaluateGradient(const VectorX& x) override;
	void EvaluateHessian(const VectorX& x) override;
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
	int mi0;
	Vector3 mp;

	//For parallization
	T 		mE;
	Vector3 mg;
	Matrix3 mH;
	Vector3 md;
};
}
#endif