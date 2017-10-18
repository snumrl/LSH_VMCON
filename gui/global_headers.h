#ifndef __GLOBAL_HEADERS_H__
#define __GLOBAL_HEADERS_H__
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <memory>
#include <vector>
#include <iostream>

#define DOUBLE_PRECISION
#ifdef DOUBLE_PRECISION
typedef double T;
#else
typedef float T;
#endif
typedef Eigen::Matrix<T, 2, 2, 0, 2 ,2> Matrix2;
typedef Eigen::Matrix<T, 2, 1, 0, 2 ,1> Vector2;
typedef Eigen::Matrix<T, 3, 3, 0, 3 ,3> Matrix3;
typedef Eigen::Matrix<T, 3, 1, 0, 3 ,1> Vector3;
typedef Eigen::Matrix<T, 4, 4, 0, 4 ,4> Matrix4;
typedef Eigen::Matrix<T, 4, 1, 0, 4 ,1> Vector4;
typedef Eigen::Matrix<T, 12, 12, 0, 12 ,12> Matrix12;
typedef Eigen::Matrix<T, 12, 1, 0, 12 ,1> Vector12;
typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorX;
typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
typedef Eigen::SparseMatrix<T> SMatrix;
typedef Eigen::Triplet<T,int> SMatrixTriplet;
typedef std::vector<Vector3,Eigen::aligned_allocator<Vector3>> EigenVector3Vector;
typedef Eigen::Quaternion<T> Quater;


#define block3(a) block<3,1>(3*(a),0)

#endif