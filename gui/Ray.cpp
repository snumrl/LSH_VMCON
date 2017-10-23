#include "Ray.h"
using namespace GUI;
Ray::
Ray(const Eigen::Vector3d& _p0,const Eigen::Vector3d& _v)
	:p0(_p0),v(_v)
{
}

void
Ray::
GetDistance(const Eigen::Vector3d& x,double& t,double& d)
{
	t = (v.dot(x-p0))/(v.dot(v));
	Eigen::Vector3d p_t = p0 + t*v;
	d = (x-p_t).norm();
}
