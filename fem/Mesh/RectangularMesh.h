#ifndef __RECTANGULAR_MESH_H__
#define __RECTANGULAR_MESH_H__

#include "Mesh.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
namespace FEM
{

class RectangularMesh : public Mesh
{
public:
	RectangularMesh(double _x = 1.0,double _y = 1.0, double _z = 1.0, int _nx = 5,int _ny = 5, int _nz = 5,const Eigen::Isometry3d& M = Eigen::Isometry3d::Identity());

protected:
	int mNx,mNy,mNz;
	double mX,mY,mZ;
};
};
#endif