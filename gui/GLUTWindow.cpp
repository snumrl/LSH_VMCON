#include "GLUTWindow.h"
#include "Camera.h"
#include <GL/glut.h>
using namespace GUI;
std::vector<GLUTWindow*> GLUTWindow::mWindows;
std::vector<int> GLUTWindow::mWinIDs;

GLUTWindow::
GLUTWindow()
	:mCamera(new Camera()),mIsDrag(false),mMouseType(0),mPrevX(0),mPrevY(0),mDisplayTimeout(1.0/30.0)
{

}
GLUTWindow::
~GLUTWindow()
{

}

void
GLUTWindow::
InitWindow(int _w,int _h,const char* _name)
{
	mWindows.push_back(this);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE | GLUT_ACCUM);
	glutInitWindowPosition(150, 100);
	glutInitWindowSize(_w, _h);
	mWinIDs.push_back(glutCreateWindow(_name));
	glutDisplayFunc(DisplayEvent);
	glutReshapeFunc(ReshapeEvent);
	glutKeyboardFunc(KeyboardEvent);
	glutMouseFunc(MouseEvent);
	glutMotionFunc(MotionEvent);
	glutTimerFunc(mDisplayTimeout, TimerEvent, 0);
}
inline GLUTWindow*
GLUTWindow::
current()
{
	int id = glutGetWindow();
	for (int i = 0; i < mWinIDs.size(); i++)
	{
		if (mWinIDs.at(i) == id) {
			return mWindows.at(i);
		}
	}
	std::cout << "An unknown error occurred!" << std::endl;
	exit(0);
}
void
GLUTWindow::
DisplayEvent()
{
	current()->Display();
}
void
GLUTWindow::
KeyboardEvent(unsigned char key,int x,int y)
{
	current()->Keyboard(key,x,y);
}
void
GLUTWindow::
MouseEvent(int button, int state, int x, int y)
{
	current()->Mouse(button,state,x,y);
}
void
GLUTWindow::
MotionEvent(int x, int y)
{
	current()->Motion(x,y);
}
void
GLUTWindow::
ReshapeEvent(int w, int h)
{
	current()->Reshape(w,h);
}
void
GLUTWindow::
TimerEvent(int value)
{
	current()->Timer(value);
}

