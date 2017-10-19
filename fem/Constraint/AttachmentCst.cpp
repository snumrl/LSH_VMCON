#include "AttachmetnCst.h"

using namespace FEM;

AttachmentCst::
AttachmentCst(T k,int i0,const Vector3& p)
	:Cst(k),mi0(i0),mp(p)
{
	mE = 0.0;
	mg.setZero();
	mH.setZero();
	md.setZero();
}
void
AttachmentCst::
EvaluatePotentialEnergy(const VectorX& x)
{
	mE = 0.5 * mStiffness * ((x.block(mi0) - mp).squaredNorm());
}
void
AttachmentCst::
EvaluateGradient(const VectorX& x)
{
	mg = mStiffness*(x.block(mi0) - mp);
}
void
AttachmentCst::
EvaluateHessian(const VectorX& x)
{
	Vector3 kv;

	kv[0] = mStiffness;
	kv[1] = mStiffness;
	kv[2] = mStiffness;
	mH = kv.asDiagonal();
}
void
AttachmentCst::
GetPotentialEnergy(T& e)
{
	e += mE;
}
void
AttachmentCst::
GetGradient(VectorX& g)
{
	g.block(mi0) += mg;
}
void
AttachmentCst::
GetHessian(std::vector<SMatrixTriplet>& h_triplets)
{
	h_triplets.push_back(SMatrixTriplet(3*mi0+0, 3*mi0+0, mH(0, 0)));
	h_triplets.push_back(SMatrixTriplet(3*mi0+1, 3*mi0+1, mH(1, 1)));
	h_triplets.push_back(SMatrixTriplet(3*mi0+2, 3*mi0+2, mH(2, 2)));
}
void
AttachmentCst::
EvaluateDVector(const VectorX& x)
{
	md = mp;
}
void
AttachmentCst::
GetDVector(int& index,VectorX& d)
{
	d.block(index) = md;
	index++;
}
void
AttachmentCst::
EvaluateJMatrix(int& index, std::vector<SMatrixTriplet>& J_triplets)
{
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+0, 3*index+0, mStiffness));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+1, 3*index+1, mStiffness));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+2, 3*index+2, mStiffness));
	index++;
}
void
AttachmentCst::
EvaluateLMatrix(std::vector<SMatrixTriplet>& L_triplets)
{
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+0, 3*mi0+0, mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+1, 3*mi0+1, mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+2, 3*mi0+2, mStiffness));
}
int
AttachmentCst::
GetNumHessianTriplets()
{
	return 3;
}

CstType
AttachmentCst::
GetType()
{
	return CstType::ATTACHMENT;
}
void
AttachmentCst::
AddOffset(int offset)
{
	mi0 += offset;
}