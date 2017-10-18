#ifndef __GUI_CAMERA_H__
#define __GUI_CAMERA_H__
#include "global_headers.h"

namespace GUI
{
class Camera
{
public:
	Camera();
		
	void SetCamera(const Vector3& lookAt,const Vector3& eye,const Vector3& up);
	void Apply();

	void Pan(int x,int y,int prev_x,int prev_y);
	void Zoom(int x,int y,int prev_x,int prev_y);
	void Rotate(int x,int y,int prev_x,int prev_y);
	void Translate(int x,int y,int prev_x,int prev_y);
private:
	Vector3 lookAt;
	Vector3 eye;
	Vector3 up;
	T fovy;

	Vector3 Rotateq(const Vector3& target, const Vector3& rotateVector,T angle);
	Vector3 GetTrackballPoint(int mouseX, int mouseY,int w,int h);
	Vector3 UnProject(const Vector3& vec);
};

};

#endif