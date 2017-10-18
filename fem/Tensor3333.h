#ifndef __FEM_TENSOR_3333_H__
#define __FEM_TENSOR_3333_H__
#include "global_headers.h"
#include <iostream>
namespace FEM
{
class Tensor3333
{
public:
	Tensor3333();
	Tensor3333(const Tensor3333& other);
	Tensor3333& operator=(const Tensor3333& other);

	Tensor3333 operator+() const;
	Tensor3333 operator-() const;
	Tensor3333 operator+(const Tensor3333& B) const;
	Tensor3333 operator-(const Tensor3333& B) const;
	Tensor3333 operator*(const Matrix3& m) const;
	Tensor3333 operator*(T a) const;

	Matrix3& operator()(int i, int j);

	void SetIdentity();
	void SetZero();
	Tensor3333 Transpose();

public:
	Matrix3 A[3][3];
};

Tensor3333 operator*(T a,const Tensor3333& B);
Tensor3333 operator*(const Matrix3& m,const Tensor3333& B);
std::ostream& operator<<(std::ostream& os,const Tensor3333& B);
}
#endif