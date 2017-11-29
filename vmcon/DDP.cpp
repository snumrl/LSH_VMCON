#include "DDP.h"
#include <iostream>
#include <fstream>

DDP::
DDP(int sx,int su,int n,int max_iteration)
	:mSx(sx),mSu(su),mN(n),mMaxIteration(max_iteration),
	mMu(1.0),mMu_min(1E-6),mMu_max(1E10),mLambda(1.0),mLambda_0(2.0),mAlpha(1.0)
{
	mx.resize(mN,Eigen::VectorXd::Zero(mSx));
	mu.resize(mN-1,Eigen::VectorXd::Zero(mSu));

	mu_lower = Eigen::VectorXd::Zero(mSu);
	mu_upper = Eigen::VectorXd::Zero(mSu);
	
	mCx.resize(mN-1,Eigen::VectorXd::Zero(mSx));	
	mCu.resize(mN-1,Eigen::VectorXd::Zero(mSu));	
	mCxx.resize(mN-1,Eigen::MatrixXd::Zero(mSx,mSx));
	mCxu.resize(mN-1,Eigen::MatrixXd::Zero(mSx,mSu));
	mCuu.resize(mN-1,Eigen::MatrixXd::Zero(mSu,mSu));

	mfx.resize(mN-1,Eigen::MatrixXd::Zero(mSx,mSx));
	mfu.resize(mN-1,Eigen::MatrixXd::Zero(mSx,mSu));

	mK.resize(mN-1,Eigen::MatrixXd::Zero(mSu,mSx));
	mk.resize(mN-1,Eigen::VectorXd::Zero(mSu));

	mVx.resize(mN,Eigen::VectorXd::Zero(mSx));
	mVxx.resize(mN,Eigen::MatrixXd::Zero(mSx,mSx));
}
void
DDP::
Init(const Eigen::VectorXd& x0,const std::vector<Eigen::VectorXd>& u0,const Eigen::VectorXd& u_lower,const Eigen::VectorXd& u_upper)
{
	mx[0] = x0;
	mu = u0;
	mu_lower = u_lower;
	mu_upper = u_upper;

	for(int t = 0;t <mN-1;t++)
		Evalf(mx[t],mu[t],t,mx[t+1]);
}
void
DDP::
ComputeDerivative()
{
	mCost = 0;
	for(int t =0;t<mN-1;t++)
	{
		double c;

		Evalfx(mx[t],mu[t],t,mfx[t]);
		Evalfu(mx[t],mu[t],t,mfu[t]);

		EvalC(  mx[t],mu[t],t, c);
		EvalCx( mx[t],mu[t],t, mCx[t]);
		EvalCu( mx[t],mu[t],t, mCu[t]);
		EvalCxx(mx[t],mu[t],t, mCxx[t]);
		EvalCxu(mx[t],mu[t],t, mCxu[t]);
		EvalCuu(mx[t],mu[t],t, mCuu[t]);

		mCost+=c;
	}

	double cf;
	EvalCf(mx[mN-1],cf);
	mCost +=cf;

	std::cout<<"Cost : "<<mCost<<std::endl;

	EvalCfx(mx[mN-1],mVx[mN-1]);
	EvalCfxx(mx[mN-1],mVxx[mN-1]);
}
bool
DDP::
BackwardPass()
{
	Eigen::VectorXd Qx(mSx),Qu(mSu);
	Eigen::MatrixXd Qxx(mSx,mSx),Qxu(mSx,mSu),Qxu_reg(mSx,mSu),Qux(mSu,mSx),Qux_reg(mSu,mSx);
	
	Eigen::MatrixXd Quu(mSu,mSu),Quu_reg(mSu,mSu),Quu_inv(mSu,mSu);
	Eigen::MatrixXd muI = mMu*Eigen::MatrixXd::Identity(mSx,mSx);
	mdV[0] = mdV[1] = 0.0;

	for(int t = mN-2;t>=0;t--)
	{
		Qx = mCx[t] + mfx[t].transpose()*mVx[t+1];
		Qu = mCu[t] + mfu[t].transpose()*mVx[t+1];
		
		Qxx = mCxx[t] + mfx[t].transpose()*mVxx[t+1]*mfx[t];
		Qxu = mCxu[t] + mfx[t].transpose()*(mVxx[t+1])*mfu[t];
		Qux = Qxu.transpose();
		Quu = mCuu[t] + mfu[t].transpose()*(mVxx[t+1])*mfu[t];

		Qxu_reg = mCxu[t] + mfx[t].transpose()*(mVxx[t+1]+muI)*mfu[t];
		Qux_reg = Qxu_reg.transpose();
		Quu_reg = mCuu[t] + mfu[t].transpose()*(mVxx[t+1]+muI)*mfu[t];

		if(!CheckPSD(Quu_reg))
			return false;

		Quu_inv = Quu_reg.inverse();

		
		mK[t] = -Quu_inv*Qux;
		mk[t] = -Quu_inv*Qu;
		
		mdV[0] += mk[t].transpose()*Qu;
		mdV[1] += 0.5*mk[t].transpose()*Quu*mk[t];
		mVx[t] = Qx + mK[t].transpose()*Quu*mk[t] + mK[t].transpose()*Qu + Qxu*mk[t];
		mVxx[t] = Qxx + mK[t].transpose()*Quu*mK[t] + mK[t].transpose()*Qux + Qxu*mK[t];
	}

	return true;
}
double
DDP::
ForwardPass()
{
	std::vector<Eigen::VectorXd> x_new;
	std::vector<Eigen::VectorXd> u_new;

	x_new.resize(mN,Eigen::VectorXd::Zero(mSx));
	u_new.resize(mN-1,Eigen::VectorXd::Zero(mSu));
	x_new[0] = mx[0];
	for(int t = 0;t<mN-1;t++)
	{
		u_new[t] = mu[t] + mAlpha*mk[t] + mK[t]*(x_new[t]-mx[t]);

		for(int i =0;i<u_new[t].rows();i++)
		{
			u_new[t] = u_new[t].cwiseMax(mu_lower);
			u_new[t] = u_new[t].cwiseMin(mu_upper);
		}

		Evalf(x_new[t],u_new[t],t,x_new[t+1]);
	}

	mx = x_new;
	mu = u_new;

	double cost_new = 0;
	double c = 0;
	for(int t =0;t<mN-1;t++){
		EvalC(mx[t],mu[t],t,c);
		cost_new +=c;
	}
	EvalCf(mx[mN-1],c);
	cost_new += c;

	return cost_new;
}

const std::vector<Eigen::VectorXd>&
DDP::
Solve()
{
	for(int i = 0;i<mMaxIteration;i++)
	{
		ComputeDerivative();
		
		mBackwardPassDone = false;
		while(true)
		{
			bool success = BackwardPass();
			if(success){
				mBackwardPassDone = true;
				break;
			}

			mLambda = std::max(mLambda*mLambda_0,mLambda_0);
			mMu = std::max(mMu*mLambda,mMu_min);
			if(mMu>mMu_max)
				break;
		}

		
		mForwardPassDone = false;	
		auto xtemp = mx;
		auto utemp = mu;
		mAlpha = 1.0;
		double dcost;
		if(mBackwardPassDone)
		{
			for(int k =0;k<10;k++)
			{
				mx = xtemp;	
				mu = utemp;	
				double cost_new = ForwardPass();
				dcost = mCost - cost_new;
				double expected = -mAlpha*(mdV[0] + mAlpha*mdV[1]);
				double z; 
				
				if(expected >0)
					z = dcost/expected;
				else
					z = (dcost>0? 1:-1);
				if(z>0.0){

					mForwardPassDone = true;
					break;
				}
				mAlpha *= 0.5;

			}
		}

		if(mForwardPassDone)
		{
			mLambda = std::min(mLambda/mLambda_0,1.0/mLambda_0);
			mMu = mMu*mLambda;
			mMu = (mMu>mMu_min? mMu : 0.0);

			if(dcost<1E-4)
				break;
		}
		else
		{
			mLambda = std::max(mLambda*mLambda_0,mLambda_0);
			mMu = std::max(mMu*mLambda,mMu_min);

			mx = xtemp;
			mu = utemp;

			if(mMu>mMu_max)
				break;
		}
	}	


	return mu;
}

bool
DDP::
CheckPSD(const Eigen::MatrixXd& A)
{
	Eigen::VectorXcd singular_values = A.eigenvalues();

    for(long i = 0; i < A.cols(); ++i)
    {
        if (singular_values[i].real() < 0.0)
        {
            std::cout << "no PSD" << std::endl;
            return false;
        }
    }
    return true;
}