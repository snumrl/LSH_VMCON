#ifndef __MESH_H__
#define __MESH_H__
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
namespace FEM
{
class Mesh
{
public:
	Mesh(){};
	~Mesh(){mVertices.clear();mTetrahedrons.clear();};
	virtual const std::vector<Eigen::Vector3d>& GetVertices(){return mVertices;};
	virtual const std::vector<Eigen::Vector4i>& GetTetrahedrons(){return mTetrahedrons;};

protected:
	std::vector<Eigen::Vector3d> mVertices;
	std::vector<Eigen::Vector4i> mTetrahedrons;
};
};


#endif