#include "SimWindow.h"
#include <GL/glut.h>

int v_target_from_argv;
// double mass_ball;
int main(int argc,char** argv)
{
	// mass_ball = std::stod(argv[1]);
	v_target_from_argv =std::stoi(argv[1]);
	SimWindow simwindow;
	glutInit(&argc, argv);
	simwindow.InitWindow(800,800,"vmcon");
	glutMainLoop();
}
