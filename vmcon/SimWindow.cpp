#include "SimWindow.h"
#include <GL/glut.h>
using namespace VMCON;
using namespace GUI;
using namespace FEM;
SimWindow::
SimWindow()
	:GLUTWindow()
{
	mSoftWorld = std::make_shared<FEM::World>(
		IntegrationMethod::PROJECTIVE_DYNAMICS,	//Integration Method
		1.0/120.0,							//Time Step
		100,								//Max Iteration
		Vector3(0,-9.81,0),					//Gravity
		0.999								//Damping
		);

	RectangularMesh rm(0.9,0.9,0.9,3,3,3);
	const auto& vertices = rm.GetVertices();
	const auto& tets = rm.GetTetrahedrons();

	VectorX p(vertices.size()*3);
	std::vector<std::shared_ptr<Cst>> cst_vec;
	
	for(int i =0;i<vertices.size();i++)
		p.block3(i) = vertices[i];

	for(const auto& t : tets)
	{
		Matrix3 Dm;
		Dm.block<3,1>(0,0) = p.block3(t[1])-p.block3(t[0]);	
		Dm.block<3,1>(0,1) = p.block3(t[2])-p.block3(t[0]);	
		Dm.block<3,1>(0,2) = p.block3(t[3])-p.block3(t[0]);

		cst_vec.push_back(std::make_shared<CorotateFEMCst>(
		1E4,
		0.3,
		t[0],t[1],t[2],t[3],1.0/6.0*Dm.determinant(),Dm.inverse()));
	}

	// VectorX p(12);

	// p.block3(0) = Vector3(0,0,0);
	// p.block3(1) = Vector3(1,0,0);
	// p.block3(2) = Vector3(1,0.5,0);
	// p.block3(3) = Vector3(0,0,0.5);

	// Matrix3 Dm;
	// Dm.block<3,1>(0,0) = p.block3(1)-p.block3(0);	
	// Dm.block<3,1>(0,1) = p.block3(2)-p.block3(0);	
	// Dm.block<3,1>(0,2) = p.block3(3)-p.block3(0);
	// std::vector<std::shared_ptr<Cst>> cst_vec;
	// cst_vec.push_back(std::make_shared<CorotateFEMCst>(
	// 	1E4,
	// 	0.3,
	// 	0,1,2,3,1.0/6.0*Dm.determinant(),Dm.inverse()));
	cst_vec.push_back(std::make_shared<AttachmentCst>(1E3,0,Vector3(0,0,0)));
	mSoftWorld->AddBody(p,cst_vec);
	mSoftWorld->Initialize();
}
void
SimWindow::
Display() 
{
	glClearColor(0.95, 0.95, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_DEPTH_TEST);
	mCamera->Apply();
	// glColor3f(0,0,0);
	// glLineWidth(1.0);
	// glBegin(GL_LINES);
	// for(double x=-10.0;x<=10.0;x+=0.5)
	// {
	// 	glVertex2f(x,-10.0);
	// 	glVertex2f(x,10.0);
	// }
	// for(double y=-10.0;y<=10.0;y+=0.5)
	// {
	// 	glVertex2f(-10.0,y);
	// 	glVertex2f(10.0,y);
	// }
	// glEnd();
 //    glColor3f(0,0,0);

	// glColor3f(0,0,1);
	// glLineWidth(5.0);
	// glBegin(GL_LINES);
	// glVertex2f(-40.0,-8);
	// glVertex2f(40.0,-8);
	// glEnd();

	// glColor3f(0,0,0);
 //    // DrawStringOnScreen(0.8,0.2,std::to_string(0.0),true);
	// glLineWidth(1.0);
	glColor3f(0,0,0);
	glPointSize(5.0);


	const auto& X = mSoftWorld->mPositions;
	glBegin(GL_LINES);

	for(auto& c: mSoftWorld->mConstraints)
	{
		const auto& tc = static_cast<CorotateFEMCst*>(c.get());
		auto p0 =X.block3(tc->mi0);
		auto p1 =X.block3(tc->mi1);
		auto p2 =X.block3(tc->mi2);
		auto p3 =X.block3(tc->mi3);
		glVertex3f(p0[0],p0[1],p0[2]);
		glVertex3f(p1[0],p1[1],p1[2]);

		glVertex3f(p0[0],p0[1],p0[2]);
		glVertex3f(p2[0],p2[1],p2[2]);

		glVertex3f(p0[0],p0[1],p0[2]);
		glVertex3f(p3[0],p3[1],p3[2]);

		glVertex3f(p1[0],p1[1],p1[2]);
		glVertex3f(p2[0],p2[1],p2[2]);

		glVertex3f(p1[0],p1[1],p1[2]);
		glVertex3f(p3[0],p3[1],p3[2]);

		glVertex3f(p2[0],p2[1],p2[2]);
		glVertex3f(p3[0],p3[1],p3[2]);
	}
	glEnd();
	glutSwapBuffers();
}
void
SimWindow::
Keyboard(unsigned char key,int x,int y) 
{
	switch(key)
	{
		case 's': mSoftWorld->TimeStepping();break;
		case 27: exit(0);break;
		default : break;
	}
	glutPostRedisplay();
}
void
SimWindow::
Mouse(int button, int state, int x, int y) 
{
	if (state == GLUT_DOWN)
	{
		mIsDrag = true;
		mMouseType = button;
		mPrevX = x;
		mPrevY = y;
	}
	else
	{
		mIsDrag = false;
		mMouseType = 0;
	}

	glutPostRedisplay();
}
void
SimWindow::
Motion(int x, int y) 
{
	if (!mIsDrag)
		return;

	int mod = glutGetModifiers();

	if (mMouseType == GLUT_LEFT_BUTTON)
	{
		switch (mod)
		{
		case GLUT_ACTIVE_SHIFT:
			mCamera->Translate(x,y,mPrevX,mPrevY);
			break;
		default:
			mCamera->Rotate(x,y,mPrevX,mPrevY);
			break;
		}
	}
	else if (mMouseType == GLUT_RIGHT_BUTTON)
	{
		switch (mod)
		{
		case GLUT_ACTIVE_SHIFT:
			mCamera->Zoom(x,y,mPrevX,mPrevY); break;
		default:
			mCamera->Pan(x,y,mPrevX,mPrevY); break;		
		}

	}
	mPrevX = x;
	mPrevY = y;
	glutPostRedisplay();
}
void
SimWindow::
Reshape(int w, int h) 
{
	glViewport(0, 0, w, h);
	mCamera->Apply();
}
void
SimWindow::
Timer(int value) 
{	
	// mSoftWorld->TimeStepping();
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
}