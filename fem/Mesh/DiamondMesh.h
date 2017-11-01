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
protected:
	int mStartPointIndex,mEndPointIndex;
public:
	DiamondMesh(double _x = 1.0,double _y = 1.0, double _z = 1.0, int _nx = 5,int _ny = 5, int _nz = 5,const Eigen::Affine3d& M = Eigen::Affine3d::Identity());

	int GetStartingPointIndex(){return mStartPointIndex;};
	int GetEndingPointIndex(){return mEndPointIndex;};
};
};

#endif