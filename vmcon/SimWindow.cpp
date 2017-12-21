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
	:GLUTWindow(),mIsRotate(true),
	mIsPlay(true),mIsReplay(false),mIsPaused(false),mSimTime(0.0),mRecordFrame(0),mRenderDetail(false),mRenderIK(true)
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
	if(mWorld->GetRigidWorld()->getTime()>10.0)
		Keyboard(' ',0,0);
	return mWorld->TimeStepping();
}
static Eigen::Vector3d random_target;
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

	
	DrawSkeleton(mWorld->GetMusculoSkeletalSystem()->GetSkeleton(),Eigen::Vector3d(0.8,0.8,0.8),!mRenderDetail);
	
	if(mRenderDetail)
	{
		DrawWorld(mWorld->GetSoftWorld());
		
		for(auto& mus: mWorld->GetMusculoSkeletalSystem()->GetMuscles()){
			DrawMuscleWayPoints(mus->origin_way_points);
			DrawMuscleWayPoints(mus->insertion_way_points);
			// DrawArrow3D(GetPoint(mus->origin_way_points.back()),mus->origin_force.normalized(),mus->origin_force.norm()*0.0001,0.005,Eigen::Vector3d(0,0,0));
			// DrawArrow3D(GetPoint(mus->insertion_way_points.back()),mus->insertion_force.normalized(),mus->insertion_force.norm()*0.0001,0.005,Eigen::Vector3d(0,0,0));
		}
	}
	if(mRenderIK)
	{
		auto& skel = mWorld->GetMusculoSkeletalSystem()->GetSkeleton();
		auto save_pos = skel->getPositions();
		skel->setPositions(mWorld->GetController()->GetTargetPositions());
		skel->computeForwardKinematics(true,false,false);
		DrawSkeleton(mWorld->GetMusculoSkeletalSystem()->GetSkeleton(),Eigen::Vector3d(0.8,0.2,0.2),!mRenderDetail);
		skel->setPositions(save_pos);
		skel->computeForwardKinematics(true,false,false);
	}

	Eigen::Vector3d clr[5] =
	{
		Eigen::Vector3d(0.8,0.2,0.2),
		Eigen::Vector3d(0.2,0.8,0.2), 
		Eigen::Vector3d(0.2,0.2,0.8),
		Eigen::Vector3d(0.8,0.8,0.2),
		Eigen::Vector3d(0.2,0.8,0.8)
	};
	int ball_index = 0;

	for(auto& ball : mWorld->GetBalls())
	{
		DrawSkeleton(ball->skeleton,clr[ball_index++]);
	}

	glutSwapBuffers();
}
void
SimWindow::
Keyboard(unsigned char key,int x,int y) 
{
	auto& skel = mWorld->GetMusculoSkeletalSystem()->GetSkeleton();
	Eigen::VectorXd act = mWorld->GetMusculoSkeletalSystem()->GetActivationLevels();
	random_target[0] = dart::math::random(-0.5,0.0);
	random_target[1] = dart::math::random(0.0,0.5);
	random_target[2] = dart::math::random(0.0,0.4);

	switch(key)
	{
		case '`' : mIsRotate = !mIsRotate;break;
		case 'w' : mRenderDetail = !mRenderDetail; break;
		case 'e' : mRenderIK = !mRenderIK; break;
		case 'k' : mWorld->GetController()->SetRandomTargetPositions();break;
		case '1' : act[0] += 0.1;break;
		case '2' : act[1] += 0.1;break;
		case '3' : act[2] += 0.1;break;
		case '4' : act[3] += 0.1;break;
		case '5' : act[4] += 0.1;break;
		case '6' : act[5] += 0.1;break;
		case '7' : act[6] += 0.1;break;
		case '8' : act[7] += 0.1;break;
		case '9' : act[8] += 0.1;break;
		case '0' : act[9] += 0.1;break;
		case 'r' : act.setZero();break;
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
		case 'i' : 
			// mWorld->GetController()->AddIKTarget(std::make_pair(skel->getBodyNode("HandR"),Eigen::Vector3d(0,0,0)),random_target);
			// mWorld->GetController()->SolveIK();
		break;
		case 27: exit(0);break;
		default : break;
	}
	for(int i=0;i<act.rows();i++)
		act[i] = dart::math::clip<double>(act[i],0.0,1.0);

	mWorld->GetMusculoSkeletalSystem()->SetActivationLevels(act);

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
