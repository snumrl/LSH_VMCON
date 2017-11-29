#ifndef __DDP_H__
#define __DDP_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/LU>

class DDP
{
public:
	DDP(int sx,int su,int n = 50,int max_iteration = 200);

	void Init(
		const Eigen::VectorXd& x0,const std::vector<Eigen::VectorXd>& u0,
		const Eigen::VectorXd& u_lower,
		const Eigen::VectorXd& u_upper);
public:
	virtual void EvalCf(const Eigen::VectorXd& x,double& cf) = 0;
	virtual void EvalCfx(const Eigen::VectorXd& x,Eigen::VectorXd& cfx) = 0;
	virtual void EvalCfxx(const Eigen::VectorXd& x,Eigen::MatrixXd& cfxx) = 0;

	virtual void EvalC( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,double& c) = 0;
	virtual void EvalCx( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& cx) = 0;
	virtual void EvalCu( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& cu) = 0;
	virtual void EvalCxx(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cxx) = 0;
	virtual void EvalCxu(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cxu) = 0;
	virtual void EvalCuu(const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& cuu) = 0;

	virtual void Evalf(  const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::VectorXd& f) = 0;
	virtual void Evalfx( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& fx) = 0;
	virtual void Evalfu( const Eigen::VectorXd& x,const Eigen::VectorXd& u,int t,Eigen::MatrixXd& fu) = 0;

	void ComputeDerivative();
	bool BackwardPass();
	double ForwardPass();

	const std::vector<Eigen::VectorXd>& Solve();
protected:
	bool CheckPSD(const Eigen::MatrixXd& A);
	int mMaxIteration;

	int mN;
	int mSx,mSu;

	//Output
	std::vector<Eigen::VectorXd> mx;		// dim : n		size : w
	std::vector<Eigen::VectorXd> mu;		// dim : m		size : w-1
	Eigen::VectorXd mu_lower,mu_upper;
	//Simulation Memory
	std::vector<Eigen::VectorXd> mCx;		// dim : 1*n	size : w-1 
	std::vector<Eigen::VectorXd> mCu;		// dim : 1*m	size : w-1
	std::vector<Eigen::MatrixXd> mCxx;		// dim : n*n	size : w-1
	std::vector<Eigen::MatrixXd> mCxu;		// dim : n*m	size : w-1
	std::vector<Eigen::MatrixXd> mCuu;		// dim : m*m	size : w-1

	std::vector<Eigen::MatrixXd> mfx;		// dim : n*n	size : w-1
	std::vector<Eigen::MatrixXd> mfu;		// dim : n*m	size : w-1

	//Backward Pass Memory
	std::vector<Eigen::MatrixXd> mK;		// dim : m*m 	size : w-1
	std::vector<Eigen::VectorXd> mk;		// dim : m 		size : w-1

	double mdV[2];
	std::vector<Eigen::VectorXd> mVx;		// dim : n 		size : w
	std::vector<Eigen::MatrixXd> mVxx;		// dim : n*n 	size : w

	double mMu,mMu_min,mMu_max;
	double mLambda,mLambda_0;
	double mAlpha;

	double mCost;

	bool mBackwardPassDone,mForwardPassDone;
};


#endif