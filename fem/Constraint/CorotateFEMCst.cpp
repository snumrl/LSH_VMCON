#include "CorotateFEMCst.h"
#include <Eigen/SVD>
#include <Eigen/LU>
using namespace FEM;
CorotateFEMCst::
CorotateFEMCst(T k,T poisson_ratio,int i0,int i1,int i2,int i3,T volume,const Matrix3& invDm)
	:Cst(k),mi0(i0),mi1(i1),mi2(i2),mi3(i3),mVolume(volume),mInvDm(invDm),
	mMu(k/((1.0+poisson_ratio))),mLambda(k*poisson_ratio/((1.0+poisson_ratio)*(1-2.0*poisson_ratio)))
{
	mE = 0.0;
	mg.setZero();
	mH.setZero();
	md.setZero();
	mX.setZero();
	mF.setZero();
	mR.setZero();
	mU.setZero();
	mV.setZero();
	mD.setZero();
}
void
CorotateFEMCst::
EvaluatePotentialEnergy(const VectorX& x)
{
	ComputeF(x);

	T vol_preserve = (mD-Matrix3::Identity()).trace();
	mE = mVolume*(0.5*mMu*((mF - mR).norm())+0.5*mLambda*vol_preserve*vol_preserve);
}
void
CorotateFEMCst::
EvaluateGradient(const VectorX& x)
{
	ComputeF(x);

	Matrix3 P;
	ComputeP(P);

	P = mVolume*P*(mInvDm.transpose());
	mg.block3(0) = -(P.block<3,1>(0,0) + P.block<3,1>(0,1) + P.block<3,1>(0,2));
	mg.block3(1) = P.block<3,1>(0,0);
	mg.block3(2) = P.block<3,1>(0,1);
	mg.block3(3) = P.block<3,1>(0,2);
}
void
CorotateFEMCst::
EvaluateHessian(const VectorX& x)
{
	ComputeF(x);

	Tensor3333 dPdF;

	ComputedPdF(dPdF);

	dPdF = mVolume*dPdF*mInvDm.transpose();

	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			//H_ij
			Vector3 df0,df1,df2;
			df0 = dPdF(0,j)*mInvDm.transpose().block<3,1>(0,i);
			df1 = dPdF(1,j)*mInvDm.transpose().block<3,1>(0,i);
			df2 = dPdF(2,j)*mInvDm.transpose().block<3,1>(0,i);

			mH.block<3,1>(i*3,j*3+0) = df0;
			mH.block<3,1>(i*3,j*3+1) = df1;
			mH.block<3,1>(i*3,j*3+2) = df2;
		}

		mH.block<3,3>(i*3,9) = -mH.block<3,3>(i*3,0)-mH.block<3,3>(i*3,3)-mH.block<3,3>(i*3,6);
	}
	for(int i=0;i<4;i++)
	{
		mH.block<3,3>(9,i*3) = -mH.block<3,3>(0,i*3)-mH.block<3,3>(3,i*3)-mH.block<3,3>(6,i*3);
	}
}
void
CorotateFEMCst::
GetPotentialEnergy(T& e)
{
	e += mE;
}
void
CorotateFEMCst::
GetGradient(VectorX& g)
{
	g.block3(mi0) += mg.block3(0);
	g.block3(mi1) += mg.block3(1);
	g.block3(mi2) += mg.block3(2);
	g.block3(mi3) += mg.block3(3);
}
void
CorotateFEMCst::
GetHessian(std::vector<SMatrixTriplet>& h_triplets)
{
	int idx[4] = {mi0,mi1,mi2,mi3};

	for(int i =0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			//H.block [i,j] --- 3x3 matrix
			h_triplets.push_back(SMatrixTriplet(3*idx[i]+0, 3*idx[j]+0, mH(3*i+0, 3*j+0)));
			h_triplets.push_back(SMatrixTriplet(3*idx[i]+0, 3*idx[j]+1, mH(3*i+0, 3*j+1)));
			h_triplets.push_back(SMatrixTriplet(3*idx[i]+0, 3*idx[j]+2, mH(3*i+0, 3*j+2)));
			h_triplets.push_back(SMatrixTriplet(3*idx[i]+1, 3*idx[j]+0, mH(3*i+1, 3*j+0)));
			h_triplets.push_back(SMatrixTriplet(3*idx[i]+1, 3*idx[j]+1, mH(3*i+1, 3*j+1)));
			h_triplets.push_back(SMatrixTriplet(3*idx[i]+1, 3*idx[j]+2, mH(3*i+1, 3*j+2)));
			h_triplets.push_back(SMatrixTriplet(3*idx[i]+2, 3*idx[j]+0, mH(3*i+2, 3*j+0)));
			h_triplets.push_back(SMatrixTriplet(3*idx[i]+2, 3*idx[j]+1, mH(3*i+2, 3*j+1)));
			h_triplets.push_back(SMatrixTriplet(3*idx[i]+2, 3*idx[j]+2, mH(3*i+2, 3*j+2)));
		}
	}
}
void
CorotateFEMCst::
EvaluateDVector(const VectorX& x)
{
	ComputeF(x);
	md = mR;
}
void
CorotateFEMCst::
GetDVector(int& index,VectorX& d)
{
	d.block3(index) = md.block<3,1>(0,0);
	d.block3(index+1) = md.block<3,1>(0,1);
	d.block3(index+2) = md.block<3,1>(0,2);
	index+=3;
}
void
CorotateFEMCst::
EvaluateJMatrix(int& index, std::vector<SMatrixTriplet>& J_triplets)
{
	MatrixX Ai(3*3,3*4);
	T d11 = mInvDm(0,0);
	T d12 = mInvDm(0,1);
	T d13 = mInvDm(0,2);
	T d21 = mInvDm(1,0);
	T d22 = mInvDm(1,1);
	T d23 = mInvDm(1,2);
	T d31 = mInvDm(2,0);
	T d32 = mInvDm(2,1);
	T d33 = mInvDm(2,2);

	Ai<<
		-d11-d21,0,0,d11,0,0,d21,0,0,d31,0,0,
		0,-d11-d21,0,0,d11,0,0,d21,0,0,d31,0,
		0,0,-d11-d21,0,0,d11,0,0,d21,0,0,d31,
		-d12-d22,0,0,d12,0,0,d22,0,0,d32,0,0,
		0,-d12-d22,0,0,d12,0,0,d22,0,0,d32,0,
		0,0,-d12-d22,0,0,d12,0,0,d22,0,0,d32,
		-d13-d23,0,0,d13,0,0,d23,0,0,d33,0,0,
		0,-d13-d23,0,0,d13,0,0,d23,0,0,d33,0,
		0,0,-d13-d23,0,0,d13,0,0,d23,0,0,d33;

	auto MuAiT = mMu*mVolume*Ai.transpose();
	int idx[4] = {mi0,mi1,mi2,mi3};
	//MuAiT --- 12x9 matrix
	for(int i =0;i<4;i++)
	{
		for(int j=0;j<3;j++)
		{
			//MuAiT.block [i,j] -- 3x3 matrix
			J_triplets.push_back(SMatrixTriplet(3*idx[i]+0, 3*(index+j)+0, MuAiT(3*i+0, 3*j+0)));
			J_triplets.push_back(SMatrixTriplet(3*idx[i]+0, 3*(index+j)+1, MuAiT(3*i+0, 3*j+1)));
			J_triplets.push_back(SMatrixTriplet(3*idx[i]+0, 3*(index+j)+2, MuAiT(3*i+0, 3*j+2)));
			J_triplets.push_back(SMatrixTriplet(3*idx[i]+1, 3*(index+j)+0, MuAiT(3*i+1, 3*j+0)));
			J_triplets.push_back(SMatrixTriplet(3*idx[i]+1, 3*(index+j)+1, MuAiT(3*i+1, 3*j+1)));
			J_triplets.push_back(SMatrixTriplet(3*idx[i]+1, 3*(index+j)+2, MuAiT(3*i+1, 3*j+2)));
			J_triplets.push_back(SMatrixTriplet(3*idx[i]+2, 3*(index+j)+0, MuAiT(3*i+2, 3*j+0)));
			J_triplets.push_back(SMatrixTriplet(3*idx[i]+2, 3*(index+j)+1, MuAiT(3*i+2, 3*j+1)));
			J_triplets.push_back(SMatrixTriplet(3*idx[i]+2, 3*(index+j)+2, MuAiT(3*i+2, 3*j+2)));
		}
	}
	index+=3;
}
void
CorotateFEMCst::
EvaluateLMatrix(std::vector<SMatrixTriplet>& L_triplets)
{
	MatrixX Ai(3*3,3*4);
	T d11 = mInvDm(0,0);
	T d12 = mInvDm(0,1);
	T d13 = mInvDm(0,2);
	T d21 = mInvDm(1,0);
	T d22 = mInvDm(1,1);
	T d23 = mInvDm(1,2);
	T d31 = mInvDm(2,0);
	T d32 = mInvDm(2,1);
	T d33 = mInvDm(2,2);

	Ai<<
		-d11-d21,0,0,d11,0,0,d21,0,0,d31,0,0,
		0,-d11-d21,0,0,d11,0,0,d21,0,0,d31,0,
		0,0,-d11-d21,0,0,d11,0,0,d21,0,0,d31,
		-d12-d22,0,0,d12,0,0,d22,0,0,d32,0,0,
		0,-d12-d22,0,0,d12,0,0,d22,0,0,d32,0,
		0,0,-d12-d22,0,0,d12,0,0,d22,0,0,d32,
		-d13-d23,0,0,d13,0,0,d23,0,0,d33,0,0,
		0,-d13-d23,0,0,d13,0,0,d23,0,0,d33,0,
		0,0,-d13-d23,0,0,d13,0,0,d23,0,0,d33;

	auto MuAiTAi = mMu*mVolume*((Ai.transpose())*Ai);
	int idx[4] = {mi0,mi1,mi2,mi3};
	//MuAiT --- 12x12 matrix
	for(int i =0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			//MuAiTAi.block [i,j] -- 3x3 matrix
			L_triplets.push_back(SMatrixTriplet(3*idx[i]+0, 3*idx[j]+0, MuAiTAi(3*i+0, 3*j+0)));
			L_triplets.push_back(SMatrixTriplet(3*idx[i]+0, 3*idx[j]+1, MuAiTAi(3*i+0, 3*j+1)));
			L_triplets.push_back(SMatrixTriplet(3*idx[i]+0, 3*idx[j]+2, MuAiTAi(3*i+0, 3*j+2)));
			L_triplets.push_back(SMatrixTriplet(3*idx[i]+1, 3*idx[j]+0, MuAiTAi(3*i+1, 3*j+0)));
			L_triplets.push_back(SMatrixTriplet(3*idx[i]+1, 3*idx[j]+1, MuAiTAi(3*i+1, 3*j+1)));
			L_triplets.push_back(SMatrixTriplet(3*idx[i]+1, 3*idx[j]+2, MuAiTAi(3*i+1, 3*j+2)));
			L_triplets.push_back(SMatrixTriplet(3*idx[i]+2, 3*idx[j]+0, MuAiTAi(3*i+2, 3*j+0)));
			L_triplets.push_back(SMatrixTriplet(3*idx[i]+2, 3*idx[j]+1, MuAiTAi(3*i+2, 3*j+1)));
			L_triplets.push_back(SMatrixTriplet(3*idx[i]+2, 3*idx[j]+2, MuAiTAi(3*i+2, 3*j+2)));
		}
	}

}
int
CorotateFEMCst::
GetNumHessianTriplets()
{
	return 144;
}

CstType
CorotateFEMCst::
GetType()
{
	return COROTATE_FEM;
}
void
CorotateFEMCst::
AddOffset(int offset)
{
	mi0 +=offset;
	mi1 +=offset;
	mi2 +=offset;
	mi3 +=offset;
}
void
CorotateFEMCst::
ComputeF(const VectorX& x)
{
	mX.block3(0) = x.block3(mi0);
	mX.block3(1) = x.block3(mi1);
	mX.block3(2) = x.block3(mi2);
	mX.block3(3) = x.block3(mi3);

	Matrix3 Ds;
	Ds.block<3,1>(0,0) = mX.block3(1) - mX.block3(0);
	Ds.block<3,1>(0,1) = mX.block3(2) - mX.block3(0);
	Ds.block<3,1>(0,2) = mX.block3(3) - mX.block3(0);

	mF = Ds*mInvDm;

	Eigen::JacobiSVD<Matrix3> svd(mF, Eigen::ComputeFullU | Eigen::ComputeFullV);

	mD(0,0) = svd.singularValues()[0];
	mD(1,1) = svd.singularValues()[1];
	mD(2,2) = svd.singularValues()[2];

	mU = svd.matrixU();
	mV = svd.matrixV();
	mR = mU*mV.transpose();
}

void
CorotateFEMCst::
ComputeP(Matrix3& P)
{
	P = mMu*(mF - mR) +
		mLambda*((mR.transpose()*mF-Matrix3::Identity()).trace())*mR;
}
void
CorotateFEMCst::
ComputedPdF(Tensor3333& dPdF)
{
	Tensor3333 dFdF,dRdF;
	dFdF.SetIdentity();

	//Compute dRdF
	for(int i =0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			Matrix3 M = mU.transpose()*dFdF(i,j)*mV;
			if(fabs(mD(0,0)-mD(1,1))<BIG_EPSILON && fabs(mD(0,0)-mD(2,2))<BIG_EPSILON)
			{
				Matrix3 off_diag_M;
				off_diag_M.setZero();
				for(int a=0;a<3;a++)
					for(int b=0;b<3;b++)		
					{
						if(a==b)
							continue;
						else
							off_diag_M(a,b) = M(a,b) / mD(0,0);
					}

				dRdF(i,j) = mU*off_diag_M*mV.transpose();
			}
			else
			{
				Vector2 unknown_side, known_side;
				Matrix2 known_matrix;
				Matrix3 U_tilde, V_tilde;
				U_tilde.setZero(); V_tilde.setZero();
				Matrix2 reg;
				reg.setZero();
				reg(0, 0) = reg(1, 1) = BIG_EPSILON;
				for (unsigned int row = 0; row < 3; row++)
				{
					for (unsigned int col = 0; col < row; col++)
					{
						known_side = Vector2(M(col, row), M(row, col));
						known_matrix.block<2, 1>(0, 0) = Vector2(-mD(row,row), mD(col,col));
						known_matrix.block<2, 1>(0, 1) = Vector2(-mD(col,col), mD(row,row));

						if (fabs(mD(row,row) - mD(col,col) < BIG_EPSILON))
							known_matrix += reg;
						else
							assert(fabs(known_matrix.determinant()) > EPSILON);

						unknown_side = known_matrix.inverse() * known_side;
						U_tilde(row, col) = unknown_side[0];
						U_tilde(col, row) = -U_tilde(row, col);
						V_tilde(row, col) = unknown_side[1];
						V_tilde(col, row) = -V_tilde(row, col);
					}
				}
				Matrix3 deltaU = mU*U_tilde;
				Matrix3 deltaV = V_tilde*mV.transpose();

				dRdF(i, j) = deltaU*mV.transpose() + mU*deltaV;
			}
		}
	
	Tensor3333 lambda_term;
	for(int i =0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			lambda_term(i,j) =
				(dRdF(i,j).transpose()*mF+mR.transpose()*dFdF(i,j)).trace()*mR +
				(mR.transpose()*mF-Matrix3::Identity()).trace()*dRdF(i,j);
		}

	dPdF = mMu*(dFdF - dRdF) + mLambda*lambda_term;
}
