#include "SimWindow.h"
#include <vector>
#include <string>
#include <GL/glut.h>
std::string g_state_path;
std::vector<std::string> g_record_path;
int main(int argc,char** argv)
{
	g_state_path = argv[1];
	for(int i=2;i<argc;i++)
	{
		g_record_path.push_back(argv[i]);
	}
	
	SimWindow simwindow;
	glutInit(&argc, argv);
	simwindow.InitWindow(800,800,"Render");
	glutMainLoop();
}
