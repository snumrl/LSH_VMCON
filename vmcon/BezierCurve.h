#ifndef __BEZIER_CURVE__
#define __BEZIER_CURVE__

class BezierCurve
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	BezierCurve(){};
	BezierCurve(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,double T)
		:mp0(p0),mp1(p1),mp2(p2),mInvT(1.0/T)
	{

	}

	void Initialize(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,double T)
	{
		mp0 = p0;
		mp1 = p1;
		mp2 = p2;
		mInvT = 1.0/T;
	}
	Eigen::Vector3d GetPosition(double t) const
	{
		double s = t*mInvT;
		return
			mp0*(1-s)*(1-s)+
			mp1*2*s*(1-s)+
			mp2*s*s;
	}
private:
	Eigen::Vector3d mp0,mp1,mp2;
	double mInvT;

};

#endif