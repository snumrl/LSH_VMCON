#include "SimWindow.h"
#include <algorithm>
#include <fstream>
#include <GL/glut.h>
using namespace GUI;
using namespace FEM;

using namespace dart::simulation;
using namespace dart::dynamics;
SimWindow::
SimWindow()
	:GLUTWindow(),mIsRotate(true),mIsDrag(false),mIsPlay(false),mFrame(0)
{
	std::string state_path = "../output/world_state.xml";
	mWorld = std::make_shared<IntegratedWorld>(state_path);
	for(int i =1;i<7;i++){
		LoadFromFolder("../output_"+std::to_string(i)+"/");
		mIsRender.push_back(true);
	}

	mDisplayTimeout = 33;
}

void
SimWindow::
Display() 
{
	glClearColor(0.95, 0.95, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	initLights();
	mCamera->Apply();
	glDisable(GL_LIGHTING);
	// glLineWidth(2.0);
	// DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(100,0,0),Eigen::Vector3d(1,0,0));
	// DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,100,0),Eigen::Vector3d(0,1,0));
	// DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,100),Eigen::Vector3d(0,0,1));
	// glLineWidth(1.0);
	glColor3f(0,0,0);
	glLineWidth(1.0);
	glBegin(GL_LINES);
	{
		double z = 0.0;
		for(double x=-2.0;x<=2.0;x+=0.1)
		{
			glVertex3f(x,-1.0,z);
			glVertex3f(x,3.0,z);
		}
		for(double y=-1.0;y<=3.0;y+=0.1)
		{
			glVertex3f(-2.0,y,z);
			glVertex3f(2.0,y,z);
		}
	}
	glEnd();
    glColor3f(0,0,0);
    glEnable(GL_LIGHTING);

    GUI::DrawStringOnScreen(0.8,0.2,std::to_string(mWorld->GetRigidWorld()->getTime()),true,Eigen::Vector3d(0,0,0));
	// glLineWidth(1.0);
	for(int k = 0;k<mRecords.size();k++)
	{
		if(mIsRender[k])
		{


		mRecords[k][mFrame]->Get(mWorld->GetRigidWorld(),mWorld->GetSoftWorld());
		// DrawWorld(mWorld->GetSoftWorld());

    	for(int i =0;i<mWorld->GetRigidWorld()->getNumSkeletons();i++)
    	{
    		if(i==1)
    			DrawSkeleton(mWorld->GetRigidWorld()->getSkeleton(i),Eigen::Vector3d(0.8,0.2,0.2));
    		else
    			DrawSkeleton(mWorld->GetRigidWorld()->getSkeleton(i));
	    }
	    }
	}
	
	glutSwapBuffers();
}
void
SimWindow::
Keyboard(unsigned char key,int x,int y) 
{
	switch(key)
	{

case '1' : mIsRender[0] = !mIsRender[0];break;
case '2' : mIsRender[1] = !mIsRender[1];break;
case '3' : mIsRender[2] = !mIsRender[2];break;
case '4' : mIsRender[3] = !mIsRender[3];break;
case '5' : mIsRender[4] = !mIsRender[4];break;
case '6' : mIsRender[5] = !mIsRender[5];break;
case '7' : mIsRender[6] = !mIsRender[6];break;
case '8' : mIsRender[7] = !mIsRender[7];break;
case '9' : mIsRender[8] = !mIsRender[8];break;



		case ' ' : mIsPlay =!mIsPlay;break;
		case '[' : mFrame--;break;
		case ']' : mFrame++;break;
		case 27: exit(0);break;
		default : break;
	}
	if(mFrame<0)
		mFrame = mRecords[0].size()-1;
	if(mFrame>mRecords[0].size()-1)
		mFrame= 0;

	for(int i =0;i<mIsRender.size();i++)
	{
		std::cout<<mIsRender[i]<<" ";
	}
	std::cout<<std::endl;
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
		if(!mIsRotate)
		mCamera->Translate(x,y,mPrevX,mPrevY);
		else
		mCamera->Rotate(x,y,mPrevX,mPrevY);
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
	

	if(mIsPlay){
			mFrame+=33;
			if(mFrame>mRecords[0].size())
				mFrame = 0;
		}
		
		
	
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
}


void
SimWindow::
LoadFromFolder(const std::string& path)
{

	std::vector<std::shared_ptr<Record>> records;
	for(int i=0;i<2000;i++)
	{
		std::string real_path = path+std::to_string(i);
		records.push_back(Record::Create());
		records.back()->LoadFromFile(real_path);
	}
	mRecords.push_back(records);
}
