#ifndef __RECTANGULAR_MESH_H__
#define __RECTANGULAR_MESH_H__

#include "Mesh.h"
#include "../global_headers.h"
namespace FEM
{

class RectangularMesh : public Mesh
{
public:
	RectangularMesh(T _x = 1.0,T _y = 1.0, T _z = 1.0, int _nx = 5,int _ny = 5, int _nz = 5,const Isometry3& M = Isometry3::Identity());

protected:
	int mNx,mNy,mNz;
	T mX,mY,mZ;
};
};
#endif