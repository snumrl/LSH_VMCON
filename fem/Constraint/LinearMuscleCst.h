#ifndef __LINEAR_MUSCLE_CONSTRAINT_H__
#define __LINEAR_MUSCLE_CONSTRAINT_H__
#include "../Tensor3333.h"
#include "Cst.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
namespace Eigen {

typedef Matrix<double, 12, 1> Vector12d;
typedef Matrix<double, 12, 12> Matrix12d;
};
namespace FEM
{


class LinearMuscleCst : public Cst
{
public:
	LinearMuscleCst(double k,int i0,int i1,int i2,int i3,double volume,const Eigen::Matrix3d& invDm,const Eigen::Vector3d& fiber_direction);
	void EvaluatePotentialEnergy(const Eigen::VectorXd& x) override;
	void EvaluateGradient(const Eigen::VectorXd& x) override;
	void EvaluateHessian(const Eigen::VectorXd& x) override;
	void GetPotentialEnergy(double& e) override;
	void GetGradient(Eigen::VectorXd& g) override;
	void GetHessian(std::vector<Eigen::Triplet<double>>& h_triplets) override;
	void EvaluateDVector(const Eigen::VectorXd& x) override;
	void GetDVector(int& index,Eigen::VectorXd& d) override;
	void EvaluateJMatrix(int& index, std::vector<Eigen::Triplet<double>>& J_triplets) override;
	void EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets) override;
	int GetNumHessianTriplets() override;
	void AddOffset(int offset) override;

	int GetI0() {return mi0;}
	int GetI1() {return mi1;}
	int GetI2() {return mi2;}
	int GetI3() {return mi3;}

	double GetActivationLevel() {return mActivationLevel;}
	void SetActivationLevel(double a) { mActivationLevel = a;}
protected:
	int mi0,mi1,mi2,mi3;
	double mVolume;
	Eigen::Matrix3d mInvDm;
	Eigen::Vector3d mFiberDirection;
	double mActivationLevel;
	//For parallization
	double 		mE;
	Eigen::Vector12d mg;
	Eigen::Matrix12d mH;
	Eigen::Matrix3d md;

	//For Cache
	Eigen::Vector12d mX;
	Eigen::Matrix3d mF;
	Eigen::Vector3d mp0;
	
	void ComputeF(const Eigen::VectorXd& x);
	void ComputeP(Eigen::Matrix3d& P);
	void ComputedPdF(Tensor3333& dPdF);
	void Computep0();
};
};

#endif
