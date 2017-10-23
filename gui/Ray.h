#ifndef __GUI_RAY_H__
#define __GUI_RAY_H__
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
namespace GUI
{
class Ray
{
private:
	Eigen::Vector3d p0,v;
public:
	//ray = p0 + t*v;
	Ray(const Eigen::Vector3d& p0,const Eigen::Vector3d& v);

	void GetDistance(const Eigen::Vector3d& x,double& t,double& d);
};
}
#endif