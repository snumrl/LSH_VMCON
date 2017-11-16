#ifndef __BEZIER_CURVE__
#define __BEZIER_CURVE__

class BezierCurve
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	BezierCurve(
			const Eigen::Vector3d& p0,
			const Eigen::Vector3d& p1,
			const Eigen::Vector3d& p2,
			const Eigen::Vector3d& p3)
		:mp0(p0),mp1(p1),mp2(p2),mp3(p3)
	{

	}

	Eigen::Vector3d GetPoint(double t) 
	{
		return
			(1.0-t)*(1.0-t)*(1.0-t)*mp0 + 
			3*(1.0-t)*(1.0-t)*t*mp1 +
			3*(1.0-t)*t*t*mp2 +
			t*t*t*mp3;
	}
private:
	Eigen::Vector3d mp0,mp1,mp2,mp3;

};

#endif