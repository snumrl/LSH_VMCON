#include "iLQR.h"
#include "BoxQP.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
using namespace Ipopt;
iLQR::
iLQR(int max_iteration)
	:mSx(0),mSu(0),mN(0),mMaxIteration(max_iteration),
	mMu(1.0),mMu_min(1E-6),mMu_max(1E10),mLambda(1.0),mLambda_0(2.0),mAlpha(1.0)
{
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
Init(int n,int sx_tilda,const Eigen::VectorXd& x0,const std::vector<Eigen::VectorXd>& u0,const Eigen::VectorXd& u_lower,const Eigen::VectorXd& u_upper)
{
	mSx = x0.rows();
	mSx_tilda = sx_tilda;
	mSu = u0[0].rows();

	std::cout<<"Window size : "<<n<<std::endl;
	std::cout<<"State size : "<<mSx<<std::endl;
	std::cout<<"control size : "<<mSu<<std::endl;
	std::cout<<std::endl;
	mMu = 1.0;
	mLambda = 1.0;
	mN = n;
	mx.resize(mN,Eigen::VectorXd::Zero(mSx));
	mu.resize(mN-1,Eigen::VectorXd::Zero(mSu));

	mu_lower = Eigen::VectorXd::Zero(mSu);
	mu_upper = Eigen::VectorXd::Zero(mSu);
	
	mCx.resize(mN-1,Eigen::VectorXd::Zero(mSx_tilda));	
	mCu.resize(mN-1,Eigen::VectorXd::Zero(mSu));	
	mCxx.resize(mN-1,Eigen::MatrixXd::Zero(mSx_tilda,mSx_tilda));
	mCxu.resize(mN-1,Eigen::MatrixXd::Zero(mSx_tilda,mSu));
	mCuu.resize(mN-1,Eigen::MatrixXd::Zero(mSu,mSu));

	mfx.resize(mN-1,Eigen::MatrixXd::Zero(mSx_tilda,mSx_tilda));
	mfu.resize(mN-1,Eigen::MatrixXd::Zero(mSx_tilda,mSu));
 
	mK.resize(mN-1,Eigen::MatrixXd::Zero(mSu,mSx_tilda));
	mk.resize(mN-1,Eigen::VectorXd::Zero(mSu));

	mVx.resize(mN,Eigen::VectorXd::Zero(mSx_tilda));
	mVxx.resize(mN,Eigen::MatrixXd::Zero(mSx_tilda,mSx_tilda));


	mx[0] = x0;
	mu = u0;
	mu_lower = u_lower;
	mu_upper = u_upper;

	// mCost = 0;
	// double c,cf;
	// for(int t = 0;t <mN-1;t++){
	// 	Evalf(mx[t],mu[t],t,mx[t+1]);
	// 	EvalC(  mx[t],mu[t],t, c);
	// 	mCost += c;
	// }
	// EvalCf(mx[mN-1],cf);
	// mCost +=cf;
	// std::cout<<"Cost : "<<mCost<<"(cf : "<<cf<<")"<<std::endl;
	int i=mMaxIteration;
	Finalize(i);
}
void
iLQR::
ComputeDerivative()
{
	// mCost = 0;
	double c,cf;
	
    
    
	
	for(int t =0;t<mN-1;t++)
	{
		// auto start = std::chrono::system_clock::now();
		// start = std::chrono::system_clock::now();
		// auto end = std::chrono::system_clock::now();
		// std::chrono::duration<double> elapsed_seconds = end-start;
		// start = std::chrono::system_clock::now();
		Evalf(mx[t],mu[t],t,mx[t+1]);
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"Evalf :"<<elapsed_seconds.count() << "s"<<std::endl;
		// start = std::chrono::system_clock::now();
		Evalfx(mx[t],mu[t],t,mfx[t]);
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"Evalfx :"<<elapsed_seconds.count() << "s"<<std::endl;
		// start = std::chrono::system_clock::now();
		Evalfu(mx[t],mu[t],t,mfu[t]);
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"Evalfu :"<<elapsed_seconds.count() << "s"<<std::endl;
		// start = std::chrono::system_clock::now();
		EvalCx( mx[t],mu[t],t, mCx[t]);
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"EvalCx :"<<elapsed_seconds.count() << "s"<<std::endl;
		// start = std::chrono::system_clock::now();
		EvalCu( mx[t],mu[t],t, mCu[t]);
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"EvalCu :"<<elapsed_seconds.count() << "s"<<std::endl;
		// start = std::chrono::system_clock::now();
		EvalCxx(mx[t],mu[t],t, mCxx[t]);
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"EvalCxx :"<<elapsed_seconds.count() << "s"<<std::endl;
		// start = std::chrono::system_clock::now();
		EvalCxu(mx[t],mu[t],t, mCxu[t]);
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"EvalCxu :"<<elapsed_seconds.count() << "s"<<std::endl;
		// start = std::chrono::system_clock::now();
		EvalCuu(mx[t],mu[t],t, mCuu[t]);
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"EvalCuu :"<<elapsed_seconds.count() << "s"<<std::endl;

	}

	
		


	// EvalCf(mx[mN-1],cf);
	// mCost +=cf;
	EvalCfx(mx[mN-1],mVx[mN-1]);
	EvalCfxx(mx[mN-1],mVxx[mN-1]);

}
bool
iLQR::
BackwardPass()
{
	Eigen::VectorXd Qx(mSx_tilda),Qu(mSu);
	Eigen::MatrixXd Qxx(mSx_tilda,mSx_tilda),Qxu(mSx_tilda,mSu),Qxu_reg(mSx_tilda,mSu),Qux(mSu,mSx_tilda),Qux_reg(mSu,mSx_tilda);
	
	Eigen::MatrixXd Quu(mSu,mSu),Quu_reg(mSu,mSu),Quu_inv(mSu,mSu);
	Eigen::MatrixXd muI = mMu*Eigen::MatrixXd::Identity(mSx_tilda,mSx_tilda);
	mdV[0] = mdV[1] = 0.0;

	for(int t = mN-2;t>=0;t--)
	{
		// auto start = std::chrono::system_clock::now();
		// auto end = std::chrono::system_clock::now();
		// std::chrono::duration<double> elapsed_seconds = end-start;
		// start = std::chrono::system_clock::now();
		
		Qx = mCx[t] + mfx[t].transpose()*mVx[t+1];
		Qu = mCu[t] + mfu[t].transpose()*mVx[t+1];
		
		Qxx = mCxx[t] + mfx[t].transpose()*mVxx[t+1]*mfx[t];
		Qxu = mCxu[t] + mfx[t].transpose()*(mVxx[t+1])*mfu[t];
		Qux = Qxu.transpose();
		Quu = mCuu[t] + mfu[t].transpose()*(mVxx[t+1])*mfu[t];

		Qxu_reg = mCxu[t] + mfx[t].transpose()*(mVxx[t+1]+muI)*mfu[t];
		Qux_reg = Qxu_reg.transpose();
		Quu_reg = mCuu[t] + mfu[t].transpose()*(mVxx[t+1]+muI)*mfu[t];
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"Quu_reg :"<<elapsed_seconds.count() << "s"<<std::endl;
		// if(!CheckPSD(Quu_reg)){
			// std::cout << "no PSD at "<< t<< std::endl;
			// return false;
		// }

		//For large dim
		// start = std::chrono::system_clock::now();
		Eigen::LLT<Eigen::MatrixXd> llt(Quu_reg);
		
		for(int i = 0;i<mK[t].cols();i++)
			mK[t].col(i) = -llt.solve(Qux.col(i));

		mk[t] = -llt.solve(Qu);
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"llt :"<<elapsed_seconds.count() << "s"<<std::endl;
		//For small dim
		// Quu_inv = Quu_reg.inverse();


		// Ipopt::SmartPtr<Ipopt::TNLP> QP;
		

		// Eigen::VectorXd lower(mSu),upper(mSu);
		// lower = mu_lower-mu[t];
		// upper = mu_upper-mu[t];
		// QP = new BoxQP(Quu_reg,Qu,lower,upper);
		// mQPSolver->OptimizeTNLP(QP);

		// mk[t] = -Quu_inv*Qu;
		// std::cout<<Quu_inv<<std::endl;
		// std::cout<<Qu<<std::endl;
		// mk[t] = static_cast<BoxQP*>(GetRawPtr(QP))->GetSolution();
		// mK[t] = -Quu_inv*Qux;
		
		// std::cout<<mCu[t].transpose()<<std::endl;
		
		// start = std::chrono::system_clock::now();
		mdV[0] += mk[t].transpose()*Qu;
		mdV[1] += 0.5*mk[t].transpose()*Quu*mk[t];
		mVx[t] = Qx + mK[t].transpose()*Quu*mk[t] + mK[t].transpose()*Qu + Qxu*mk[t];
		mVxx[t] = Qxx + mK[t].transpose()*Quu*mK[t] + mK[t].transpose()*Qux + Qxu*mK[t];
		// end = std::chrono::system_clock::now();
		// elapsed_seconds = end-start;
		// std::cout<<"llt :"<<elapsed_seconds.count() << "s"<<std::endl;
	}

	return true;
}

double
iLQR::
ForwardPass()
{
	Eigen::VectorXd temp_x = mx[0];
	for(int t = 0;t<mN-1;t++)
	{
		mu[t] = mu[t] + mAlpha*mk[t] + mK[t]*(mx[t]-temp_x);
		// std::cout<<u_new[t].transpose()<<std::endl;
		mu[t] = mu[t].cwiseMax(mu_lower);
		mu[t] = mu[t].cwiseMin(mu_upper);

		temp_x = mx[t+1];
		Evalf(mx[t],mu[t],t,mx[t+1]);

	}

	double cost_new = 0;
	double c = 0;
	double cf = 0;
	for(int t =0;t<mN-1;t++){
		EvalC(mx[t],mu[t],t,c);
		cost_new +=c;
	}
	EvalCf(mx[mN-1],cf);
	cost_new += cf;
	return cost_new;
}
const std::vector<Eigen::VectorXd>&
iLQR::
Solve()
{
	int fail_count = 0;
	std::time_t start_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::cout << "Start " << std::ctime(&start_time)<<std::endl;

 
    
	for(int i = 0;i<mMaxIteration;i++)
	{
		auto start = std::chrono::system_clock::now();
		ComputeDerivative();
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end-start;

	    std::cout<<"ComputeDerivative : " << elapsed_seconds.count() << "s"<<std::endl;
		mBackwardPassDone = false;
		start = std::chrono::system_clock::now();
		while(true)
		{
			bool success = BackwardPass();
			if(success){
				mBackwardPassDone = true;
				break;
			}

			mLambda = std::max(mLambda*mLambda_0,mLambda_0);
			mMu = std::max(mMu*mLambda,mMu_min);
			if(mMu>mMu_max){
				// std::cout<<"During backward"<<std::endl;
				break;
			}
		}
		end = std::chrono::system_clock::now();
		elapsed_seconds = end-start;

	    std::cout<<"BackwardPass : " << elapsed_seconds.count() << "s"<<std::endl;

		mForwardPassDone = false;	
		std::vector<Eigen::VectorXd> xtemp = mx;
		std::vector<Eigen::VectorXd> utemp = mu;
		mAlpha = 1.0;
		double dcost;
		start = std::chrono::system_clock::now();
		if(mBackwardPassDone)
		{
			for(int k =0;k<10;k++)
			{
				double cost_new = ForwardPass();
				dcost = mCost - cost_new;
				double expected = -mAlpha*(mdV[0] + mAlpha*mdV[1]);
				double z; 
				
				if(expected >0)
					z = dcost/expected;
				else
					z = (dcost>0? 1:-1);
				if(z>0.03){
					mForwardPassDone = true;
					fail_count = 0;
					// std::cout<<"forward done"<<std::endl;
					// for(int t =0;t<10;t++)
						// std::cout<<mu[t].transpose()<<std::endl;
					break;
				}
				mAlpha *= 0.5;
				mx = xtemp;	
				mu = utemp;
			}
		}
		end = std::chrono::system_clock::now();
		elapsed_seconds = end-start;

	    std::cout<<"ForwardPass : " << elapsed_seconds.count() << "s"<<std::endl;
		if(mForwardPassDone)
		{
			mLambda = std::min(mLambda/mLambda_0,1.0/mLambda_0);
			mMu = mMu*mLambda;
			mMu = (mMu>mMu_min? mMu : 0.0);
			
			if(dcost<1E-4){
				// std::cout<<"dcost<1E-4"<<std::endl;
				break;
			}
		}
		else
		{
			std::cout<<"Forward Pass Fail."<<std::endl;
			mLambda = std::max(mLambda*mLambda_0,mLambda_0);
			mMu = std::max(mMu*mLambda,mMu_min);

			mx = xtemp;
			mu = utemp;

			fail_count++;
			if(fail_count == 3){
				// std::cout<<"fail_count ==3"<<std::endl;

				break;
			}
			if(mMu>mMu_max){
				// std::cout<<"During forward"<<std::endl;

				break;
			}
		}
		start = std::chrono::system_clock::now();
		Finalize(i);
		end = std::chrono::system_clock::now();
		elapsed_seconds = end-start;

	    std::cout<<"Finalize : " << elapsed_seconds.count() << "s"<<std::endl;
	    if(mCost<10.0)
	    	break;
	}
	// for(int t =0;t<10;t++)
		// std::cout<<mu[t].transpose()<<std::endl;
	// std::cout<<mu[0].transpose()<<std::endl;
	std::time_t end_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::cout << "end " << std::ctime(&end_time)<<std::endl;
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
