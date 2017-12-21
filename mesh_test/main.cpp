#include "SimWindow.h"
#include <string>
#include <GL/glut.h>
std::string obj_path;

int main(int argc,char** argv)
{
	obj_path =argv[1];
	SimWindow simwindow;
	glutInit(&argc, argv);
	simwindow.InitWindow(800,800,"vmcon");
	glutMainLoop();
}
