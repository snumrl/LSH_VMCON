#ifndef __MESH_H__
#define __MESH_H__
#include "../global_headers.h"

namespace FEM
{
class Mesh
{
public:
	Mesh(){};
	~Mesh(){mVertices.clear();mTetrahedrons.clear();};
	virtual const std::vector<Vector3>& GetVertices(){return mVertices;};
	virtual const std::vector<Eigen::Vector4i>& GetTetrahedrons(){return mTetrahedrons;};

protected:
	std::vector<Vector3> mVertices;
	std::vector<Eigen::Vector4i> mTetrahedrons;
};
};


#endif