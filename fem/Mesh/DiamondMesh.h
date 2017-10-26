#ifndef __DIAMOND_MESH_H__
#define __DIAMOND_MESH_H__

#include "RectangularMesh.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
namespace FEM
{

class DiamondMesh : public RectangularMesh
{
public:
	DiamondMesh(double _x = 1.0,double _y = 1.0, double _z = 1.0, int _nx = 5,int _ny = 5, int _nz = 5,const Eigen::Isometry3d& M = Eigen::Isometry3d::Identity());

	bool CheckInside(const Eigen::Vector3d& p);
};
};

#endif