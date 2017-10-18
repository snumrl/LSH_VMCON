#include "Camera.h"
#include <GL/glut.h>
using namespace GUI;

Camera::
Camera()
	:fovy(60.0),lookAt(Vector3(0,0,0)),eye(Vector3(0,0,10)),up(Vector3(0,1,0))
{

}
	
void
Camera::
SetCamera(const Vector3& lookAt,const Vector3& eye,const Vector3& up)
{
	this->lookAt = lookAt, this->eye = eye, this->up = up;
}
void
Camera::
Apply()
{
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fovy, (GLfloat)w / (GLfloat)h, 0.01, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eye.x(), eye.y(), eye.z(),
		lookAt.x(), lookAt.y(), lookAt.z(),
		up.x(), up.y(), up.z());
}

void
Camera::
Pan(int x,int y,int prev_x,int prev_y)
{
	T delta = (T)prev_y - (T)y;
	delta = 1 - delta / 200.0;
	eye = lookAt - (lookAt - eye)*delta;
}
void
Camera::
Zoom(int x,int y,int prev_x,int prev_y)
{
	T delta = (T)prev_y - (T)y;
	fovy += delta/20.0;
}
void
Camera::
Rotate(int x,int y,int prev_x,int prev_y)
{
	GLint w = glutGet(GLUT_WINDOW_WIDTH);
	GLint h = glutGet(GLUT_WINDOW_HEIGHT);

	Vector3 prevPoint = GetTrackballPoint(prev_x,prev_y,w,h);
	Vector3 curPoint = GetTrackballPoint(x,y,w,h);
	Vector3 rotVec = curPoint.cross(prevPoint);

	rotVec = UnProject(rotVec);
	T cosT = curPoint.dot(prevPoint) / (curPoint.norm()*prevPoint.norm());
	T sinT = (curPoint.cross(prevPoint)).norm() / (curPoint.norm()*prevPoint.norm());

	T angle = -atan2(sinT, cosT);

	Vector3 n = this->lookAt - this->eye;
	n = Rotateq(n, rotVec, angle);
	this->up = Rotateq(this->up, rotVec, angle);
	this->eye = this->lookAt - n;
}
void
Camera::
Translate(int x,int y,int prev_x,int prev_y)
{
	Vector3 delta((T)x - (T)prev_x, (T)y - (T)prev_y, 0);
	delta = UnProject(delta) / 50.0;
	lookAt += delta; eye += delta;
}
Vector3
Camera::
Rotateq(const Vector3& target, const Vector3& rotateVector,T angle)
{
	Vector3 rv = rotateVector.normalized();

	Quater rot(cos(angle / 2.0), sin(angle / 2.0)*rv.x(), sin(angle / 2.0)*rv.y(), sin(angle / 2.0)*rv.z());
	rot.normalize();
	Quater tar(0, target.x(), target.y(), target.z());


	tar = rot.inverse()*tar*rot;

	return Vector3(tar.x(), tar.y(), tar.z());
}
Vector3
Camera::
GetTrackballPoint(int mouseX, int mouseY,int w,int h)
{
	T rad = sqrt((T)(w*w+h*h)) / 2.0;
	T dx = (T)(mouseX)-(T)w / 2.0;
	T dy = (T)(mouseY)-(T)h / 2.0;
	T dx2pdy2 = dx*dx + dy*dy;

	if (rad*rad - dx2pdy2 <= 0)
		return Vector3(dx, dy, 0);
	else
		return Vector3(dx, dy, sqrt(rad*rad - dx*dx - dy*dy));
}
Vector3
Camera::
UnProject(const Vector3& vec)
{
	Vector3 n = lookAt - eye;
	n.normalize();
	
	Vector3 v = up.cross(n);
	v.normalize();

	Vector3 u = n.cross(v);
	u.normalize();

	return vec.z()*n + vec.x()*v + vec.y()*u;
}