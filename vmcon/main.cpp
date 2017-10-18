#include "SimWindow.h"
#include <GL/glut.h>

int main(int argc,char** argv)
{
	VMCON::SimWindow simwindow;
	glutInit(&argc, argv);
	simwindow.InitWindow(800,800,"vmcon");
	glutMainLoop();
}
