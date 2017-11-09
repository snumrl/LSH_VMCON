#include "SimWindow.h"
#include "MusculoSkeletalSystem.h"
#include "Controller.h"
#include "IKOptimization.h"
#include <GL/glut.h>
using namespace GUI;
using namespace FEM;

using namespace dart::simulation;
using namespace dart::dynamics;
SimWindow::
SimWindow()
	:GLUTWindow(),
	mIsPlay(false),mIsReplay(false),mIsPaused(false),mSimTime(0.0),mRecordFrame(0)
{
	dart::math::seedRand();

	mWorld = IntegratedWorld::Create();
	mWorld->Initialize();

	mDisplayTimeout = 33;
}
bool
SimWindow::
TimeStepping()
{
	return mWorld->TimeStepping();
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
	glLineWidth(2.0);
	DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(100,0,0),Eigen::Vector3d(1,0,0));
	DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,100,0),Eigen::Vector3d(0,1,0));
	DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,100),Eigen::Vector3d(0,0,1));
	glLineWidth(1.0);
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

	DrawWorld(mWorld->GetSoftWorld());
	DrawSkeleton(mWorld->GetMusculoSkeletalSystem()->GetSkeleton());
	auto& skel = mWorld->GetMusculoSkeletalSystem()->GetSkeleton();
	auto save_pos = skel->getPositions();
	DrawSkeleton(mWorld->GetMusculoSkeletalSystem()->GetSkeleton());
	skel->setPositions(mWorld->GetController()->mTargetPositions);
	skel->computeForwardKinematics(true,false,false);
	DrawSkeleton(mWorld->GetMusculoSkeletalSystem()->GetSkeleton(),Eigen::Vector3d(0.8,0.2,0.2));
	skel->setPositions(save_pos);
	skel->computeForwardKinematics(true,false,false);
	for(auto& mus: mWorld->GetMusculoSkeletalSystem()->GetMuscles()){
		DrawMuscleWayPoints(mus->origin_way_points);
		DrawMuscleWayPoints(mus->insertion_way_points);
	}

	glutSwapBuffers();
}
void
SimWindow::
Keyboard(unsigned char key,int x,int y) 
{
	switch(key)
	{
		case ' ' : mIsPlay = !mIsPlay; break;
		case 'p' : 
			if(!mIsReplay)
			{
				mIsReplay = true; mIsPlay = false; mIsPaused = true; mSimTime = mWorld->GetRigidWorld()->getTime();
			}
			else if(mIsPaused)
				mIsPaused = false;
			else
				mIsPaused = true;
			break;
		case '[' : mRecordFrame--; break;
		case ']' : mRecordFrame++; break;
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
	if(mIsPlay){
		while(!TimeStepping()){
		}
		
	}
	else if(mIsReplay){

		if(mIsPaused)
		{
			mWorld->SetRecord(mRecordFrame);

			mDisplayTimeout = 1;
		}
		else
		{
			mWorld->SetRecord(mRecordFrame);
			mRecordFrame+=33;
			mDisplayTimeout = 1000/30;	
		}
		
		
	}
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
}