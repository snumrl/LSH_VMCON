#ifndef __FEM_CST_DATA_H__
#define __FEM_CST_DATA_H__
#include "../global_headers.h"
namespace FEM
{
class Cst_Data
{
public:
	Cst_Data(T _k,int _i_cst)
		:k(_k),i_cst(_i_cst){};

	T k;
	int i_cst;
};

class AttachementCst_Data : public Cst_Data 
{
public:
	AttachementCst_Data(T _k,int _i_cst,int _i0,const Vector3& _p)
		:Cst_Data(_k,_i_cst),i0(_i0),p(_p){};

	int i0;
	Vector3 p;
};

class FEMCst_Data : public Cst_Data
{
public:
	FEMCst_Data(T _k,int _i_cst,int _i0,int _i1,int _i2,int _i3,T _mu,T _lambda,T _volume,const Matrix3& _invDm)
	:Cst_Data(0.0,_i_cst),i0(_i0),i1(_i1),i2(_i2),i3(_i3),mu(_mu),lambda(_lambda),volume(_volume),invDm(_invDm){};

	int i0,i1,i2,i3;	
	T mu,lambda;
	T volume;
	Matrix3 invDm;

};
class CorotateFEMCst_Data : public FEMCst_Data
{
public:
	CorotateFEMCst_Data(T _k,int _i_cst,int _i0,int _i1,int _i2,int _i3,T _mu,T _lambda,T _volume,const Matrix3& _invDm)
		:FEMCst_Data(_k,_i_cst,_i0,_i1,_i2,_i3,_mu,_lambda,_volume,_invDm){};


};

class LinearMuscleCst_Data : public FEMCst_Data
{
public:
	LinearMuscleCst_Data(T _k,int _i_cst,int _i0,int _i1,int _i2,int _i3,T _mu,T _lambda,T _volume,const Matrix3& _invDm,const Vector3d& _d)
		:FEMCst_Data(_k,_i_cst,_i0,_i1,_i2,_i3,_mu,_lambda,_volume,_invDm),d(_d),a(0.0){};

	Vector3 d;
	T a;
};

};
#endif