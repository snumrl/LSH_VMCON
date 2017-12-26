#include "SimWindow.h"
#include <GL/glut.h>
// int simulation_count;
int main(int argc,char** argv)
{
	// simulation_count = std::stoi(argv[1]);
	SimWindow simwindow;
	glutInit(&argc, argv);
	simwindow.InitWindow(800,800,"vmcon");
	glutMainLoop();
}
