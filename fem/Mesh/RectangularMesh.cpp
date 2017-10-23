#include "RectangularMesh.h"

using namespace FEM;
RectangularMesh::
RectangularMesh(double _x,double _y, double _z, int _nx,int _ny, int _nz,const Eigen::Isometry3d& M)
	:mNx(_nx),mNy(_ny),mNz(_nz),mX(_x),mY(_y),mZ(_z)
{


	double dx = mX /(double)mNx;
	double dy = mY /(double)mNy;
	double dz = mZ /(double)mNz;

	double x,y,z;
	for(int i=0; i<mNx+1;i++)
	{	
		x = -0.5*mNx + i*dx;
		for(int j=0; j<mNy+1;j++)
		{
			y = -0.5*mNy + j*dy;
			for(int k=0; k<mNz+1;k++)
			{
				z = -0.5*mNz + k*dz;
				mVertices.push_back(Eigen::Vector3d(x,y,z));
			}
		}
	}

	for(auto& v : mVertices)
		v = M*v;

	for(int i=0; i<mNx;i++)
	{	
		for(int j=0; j<mNy;j++)
		{
			for(int k=0; k<mNz;k++)
			{
				int index = i*(mNy+1)*(mNz+1) + j*(mNz+1)+ k;
				int cube[8] = {
					index,
					index+1,
					index+(mNz+1),
					index+(mNz+1)+1,
					index+(mNz+1)*(mNy+1),
					index+(mNz+1)*(mNy+1)+1,
					index+(mNz+1)*(mNy+1)+(mNz+1),
					index+(mNz+1)*(mNy+1)+(mNz+1)+1
				};

				mTetrahedrons.push_back(Eigen::Vector4i(cube[0],cube[1],cube[4],cube[3]));
				mTetrahedrons.push_back(Eigen::Vector4i(cube[1],cube[5],cube[4],cube[3]));
				mTetrahedrons.push_back(Eigen::Vector4i(cube[0],cube[4],cube[2],cube[3]));
				mTetrahedrons.push_back(Eigen::Vector4i(cube[3],cube[5],cube[4],cube[7]));
				mTetrahedrons.push_back(Eigen::Vector4i(cube[4],cube[6],cube[2],cube[3]));
				mTetrahedrons.push_back(Eigen::Vector4i(cube[3],cube[4],cube[6],cube[7]));
				
			}
		}
	}



	// double dx = mX /(double)mNx;
	// double dy = mY /(double)mNy;
	// double dz = mZ /(double)mNz;

	// double x,y,z;
	// for(int i=0; i<mNx+1;i++)
	// {	
	// 	x = -0.5*mNx + i*dx;
	// 	for(int j=0; i<mNy+1;j++)
	// 	{
	// 		y = -0.5*mNy + j*dy;
	// 		for(int k=0; k<mNz+1;k++)
	// 		{
	// 			z = -0.5*mNz + k*dz;
	// 			mVertices.push_back(Eigen::Vector3d(x,y,z));
	// 			if( i!=mNx && j!=mNy && k!=mNz)
	// 			{
	// 				mVertices.push_back(Eigen::Vector3d(x + 0.5*dx,y + 0.5*dy,z + 0.5*dz));
	// 			}
	// 		}
	// 	}
	// }

	// for(auto& v : mVertices)
	// 	v = M*v;

	// for(int i=0; i<mNx;i++)
	// {	
	// 	for(int j=0; i<mNy;j++)
	// 	{
	// 		for(int k=0; k<mNz;k++)
	// 		{
	// 			int index = i*(mNy+1)*(mNz+1) + i*mNy*mNz + j*(mNz+1) + j*mNz + 2*k;
	// 			int cube[9] = {
	// 				index,
	// 				index+2,
	// 				index+(2*mNz+1),
	// 				index+(2*mNz+1)+1,
	// 				index+(2*mNy*mNz+mNy+mNz+1),
	// 				index+(2*mNy*mNz+mNy+mNz+1)+2,
	// 				index+(2*mNy*mNz+mNy+mNz+1)+(2*mNz+1),
	// 				index+(2*mNy*mNz+mNy+mNz+1)+(2*mNz+1)+1,
	// 				index+1};

	// 			mTetrahedrons.push_back(Eigen::Vector4i(cube[],cube[],cube[],cube[9]));
	// 			mTetrahedrons.push_back(Eigen::Vector4i(cube[],cube[],cube[],cube[9]));
	// 			mTetrahedrons.push_back(Eigen::Vector4i(cube[],cube[],cube[],cube[9]));
	// 			mTetrahedrons.push_back(Eigen::Vector4i(cube[],cube[],cube[],cube[9]));
	// 			mTetrahedrons.push_back(Eigen::Vector4i(cube[],cube[],cube[],cube[9]));
	// 			mTetrahedrons.push_back(Eigen::Vector4i(cube[],cube[],cube[],cube[9]));

	// 		}
	// 	}
	// }
}

