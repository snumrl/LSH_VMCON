#include "iLQR.h"
#include "BoxQP.h"
#include <iostream>
#include <fstream>
using namespace Ipopt;
iLQR::
iLQR(int sx,int su,int n,int max_iteration)
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

	mQPSolver = new IpoptApplication();
	
	mQPSolver->Options()->SetStringValue("mu_strategy", "adaptive");
	mQPSolver->Options()->SetStringValue("jac_c_constant", "no");
	mQPSolver->Options()->SetStringValue("hessian_constant", "yes");
	mQPSolver->Options()->SetStringValue("mehrotra_algorithm", "yes");
	mQPSolver->Options()->SetIntegerValue("print_level", 2);
	mQPSolver->Options()->SetIntegerValue("max_iter", 10);
	mQPSolver->Options()->SetNumericValue("tol", 1e-4);

	mQPSolver->Initialize();
}
void
iLQR::
Init(const Eigen::VectorXd& x0,const std::vector<Eigen::VectorXd>& u0,const Eigen::VectorXd& u_lower,const Eigen::VectorXd& u_upper)
{
	mx[0] = x0;
	mu = u0;
	mu_lower = u_lower;
	mu_upper = u_upper;

	mCost = 0;
	double c,cf;
	for(int t = 0;t <mN-1;t++){
		Evalf(mx[t],mu[t],t,mx[t+1]);
		EvalC(  mx[t],mu[t],t, c);
		mCost += c;
	}
	EvalCf(mx[mN-1],cf);
	mCost +=cf;
	std::cout<<"Cost : "<<mCost<<"(cf : "<<cf<<")"<<std::endl;
}
void
iLQR::
ComputeDerivative()
{
	mCost = 0;
	double c,cf;
	
	for(int t =0;t<mN-1;t++)
	{
		Evalfx(mx[t],mu[t],t,mfx[t]);
		Evalfu(mx[t],mu[t],t,mfu[t]);

		EvalC(  mx[t],mu[t],t, c);
		mCost += c;
		EvalCx( mx[t],mu[t],t, mCx[t]);
		EvalCu( mx[t],mu[t],t, mCu[t]);
		EvalCxx(mx[t],mu[t],t, mCxx[t]);
		EvalCxu(mx[t],mu[t],t, mCxu[t]);
		EvalCuu(mx[t],mu[t],t, mCuu[t]);
		// std::cout<<mfx[t]<<std::endl;
		// std::cout<<mfu[t]<<std::endl;
		// std::cout<<mCu[t]<<std::endl;
		// std::cout<<mCuu[t]<<std::endl;
		// exit(0);
	}
	EvalCf(mx[mN-1],cf);
	mCost +=cf;
	std::cout<<"Cost : "<<mCost<<"(cf : "<<cf<<")"<<std::endl;
	EvalCfx(mx[mN-1],mVx[mN-1]);
	EvalCfxx(mx[mN-1],mVxx[mN-1]);

}
bool
iLQR::
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
		// std::cout<<t<<std::endl;
		// std::cout<<mMu<<std::endl;
		// std::cout<<"mfu : \n"<<mfu[t]<<std::endl;
		// std::cout<<"mfx : \n"<<mfx[t]<<std::endl;
		// std::cout<<"mVxx : \n"<<mVxx[t+1]<<std::endl;
		// std::cout<<"Qx : \n"<<Qx<<std::endl<<std::endl;
		// std::cout<<"Qu : \n"<<Qu<<std::endl<<std::endl;
		// std::cout<<"mCu : \n"<<mCu[t]<<std::endl<<std::endl;

		// std::cout<<"Qxx : \n"<<Qxx<<std::endl<<std::endl;
		// std::cout<<"Qxu : \n"<<Qxu<<std::endl<<std::endl;
		// std::cout<<"Quu : \n"<<Quu<<std::endl<<std::endl;

		// std::cout<<"Qxu_reg : \n"<<Qxu_reg<<std::endl<<std::endl;
		// std::cout<<"Quu_reg : \n"<<Quu_reg<<std::endl<<std::endl;

		if(!CheckPSD(Quu_reg)){
			// std::cout<< (mCuu[t]).eigenvalues()<<std::endl<<std::endl;
			// std::cout<< (mfu[t].transpose()*(mVxx[t+1])*mfu[t]).eigenvalues()<<std::endl<<std::endl;
			// std::cout<< (mfu[t].transpose()*(muI)*mfu[t]).eigenvalues()<<std::endl;
			// std::cout<< (mfu[t].transpose()*(mVxx[t+1]+muI)*mfu[t]).eigenvalues()<<std::endl;
			// std::cout<< (Quu_reg).eigenvalues()<<std::endl;
			// std::cout<< (mCuu[t])<<std::endl<<std::endl<<std::endl;
			// std::cout<< (mfu[t].transpose()*(mVxx[t+1])*mfu[t])<<std::endl<<std::endl<<std::endl;
			// std::cout<< mfu[t]<<std::endl<<std::endl;
			// std::cout<< (muI)<<std::endl<<std::endl;
			// std::cout<< (mVxx[t+1])<<std::endl<<std::endl;
			// std::cout<< (mVxx[t+1]+muI)<<std::endl<<std::endl;
			// std::cout<< (Quu_reg)<<std::endl<<std::endl;
			// exit(0);
			std::cout << "no PSD at "<< t<< std::endl;
			return false;
		}

		//For large dim
		// Eigen::LLT<Eigen::MatrixXd> llt(Quu_reg);
		
		// for(int i = 0;i<mK[t].cols();i++)
		// 	mK[t].col(i) = -llt.solve(Qux.col(i));

		// mK[t] = -llt.solve(Qu);

		//For small dim
		Quu_inv = Quu_reg.inverse();


		// Ipopt::SmartPtr<Ipopt::TNLP> QP;
		

		// Eigen::VectorXd lower(mSu),upper(mSu);
		// lower = mu_lower-mu[t];
		// upper = mu_upper-mu[t];
		// QP = new BoxQP(Quu_reg,Qu,lower,upper);
		// mQPSolver->OptimizeTNLP(QP);

		mk[t] = -Quu_inv*Qu;
		// std::cout<<Quu_inv<<std::endl;
		// std::cout<<Qu<<std::endl;
		// mk[t] = static_cast<BoxQP*>(GetRawPtr(QP))->GetSolution();
		mK[t] = -Quu_inv*Qux;
		
		
		mdV[0] += mk[t].transpose()*Qu;
		mdV[1] += 0.5*mk[t].transpose()*Quu*mk[t];
		mVx[t] = Qx + mK[t].transpose()*Quu*mk[t] + mK[t].transpose()*Qu + Qxu*mk[t];
		mVxx[t] = Qxx + mK[t].transpose()*Quu*mK[t] + mK[t].transpose()*Qux + Qxu*mK[t];
		// std::cout<<"mk : \n"<<mk[t].transpose()<<std::endl<<std::endl;
		// std::cout<<"mK : \n"<<mK[t]<<std::endl<<std::endl;
		// std::cout<<"mVx : \n"<<mVx[t]<<std::endl<<std::endl;
		// std::cout<<"mVxx : \n"<<mVxx[t]<<std::endl<<std::endl;
	}

	return true;
}
double
iLQR::
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
		// std::cout<<u_new[t].transpose()<<std::endl;
		u_new[t] = u_new[t].cwiseMax(mu_lower);
		u_new[t] = u_new[t].cwiseMin(mu_upper);

		Evalf(x_new[t],u_new[t],t,x_new[t+1]);

	}

	mx = x_new;
	mu = u_new;

	double cost_new = 0;
	double c = 0;
	double cf = 0;
	for(int t =0;t<mN-1;t++){
		EvalC(mx[t],mu[t],t,c);
		cost_new +=c;
	}
	EvalCf(mx[mN-1],cf);
	cost_new += cf;

	std::cout<<"alpha : "<<mAlpha<<" "<<cost_new<<"(cf : "<<cf<<")"<<std::endl;
	return cost_new;
}

const std::vector<Eigen::VectorXd>&
iLQR::
Solve()
{
	int fail_count = 0;
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
					fail_count = 0;
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
			fail_count++;
			if(fail_count == 3)
				break;
			if(dcost<1E-4)
				break;
		}
		else
		{
			std::cout<<"Forward Pass Fail."<<std::endl;
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
iLQR::
CheckPSD(const Eigen::MatrixXd& A)
{
	Eigen::VectorXcd ev = A.eigenvalues();

    for(long i = 0; i < A.cols(); ++i)
    {
        if (ev[i].real() < 0.0)
        {
            return false;
        }
    }
    return true;
}