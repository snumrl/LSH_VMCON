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
	:GLUTWindow()
{
	dart::math::seedRand();

	mWorld = IntegratedWorld::Create();
	mWorld->Initialize();

	mDisplayTimeout = 33;

		// DiamondMesh rm(2,1,1,3,1,1);
	// const auto& vertices = rm.GetVertices();
	// const auto& tets = rm.GetTetrahedrons();

	// Eigen::VectorXd p(vertices.size()*3);
	// std::vector<std::shared_ptr<Cst>> cst_vec;
	
	// for(int i =0;i<vertices.size();i++)
	// 	p.block<3,1>(i*3,0) = vertices[i];

	// for(const auto& t : tets)
	// {

	// 	Eigen::Matrix3d Dm;
	// 	Dm.block<3,1>(0,0) = p.block<3,1>(t[1]*3,0)-p.block<3,1>(t[0]*3,0);	
	// 	Dm.block<3,1>(0,1) = p.block<3,1>(t[2]*3,0)-p.block<3,1>(t[0]*3,0);	
	// 	Dm.block<3,1>(0,2) = p.block<3,1>(t[3]*3,0)-p.block<3,1>(t[0]*3,0);
	// 	// std::cout<<Dm.determinant()<<std::endl;
	// 	cst_vec.push_back(std::make_shared<CorotateFEMCst>(
	// 	1E4,
	// 	0.3,
	// 	t[0],t[1],t[2],t[3],1.0/6.0*Dm.determinant(),Dm.inverse()));
	// }

	// Eigen::VectorXd p(12);

	// p.block3(0) = Eigen::Vector3d(0,0,0);
	// p.block3(1) = Eigen::Vector3d(1,0,0);
	// p.block3(2) = Eigen::Vector3d(1,0.5,0);
	// p.block3(3) = Eigen::Vector3d(0,0,0.5);

	// Eigen::Matrix3d Dm;
	// Dm.block<3,1>(0,0) = p.block3(1)-p.block3(0);	
	// Dm.block<3,1>(0,1) = p.block3(2)-p.block3(0);	
	// Dm.block<3,1>(0,2) = p.block3(3)-p.block3(0);
	// std::vector<std::shared_ptr<Cst>> cst_vec;
	// cst_vec.push_back(std::make_shared<CorotateFEMCst>(
	// 	1E4,
	// 	0.3,
	// 	0,1,2,3,1.0/6.0*Dm.determinant(),Dm.inverse()));
	// cst_vec.push_back(std::make_shared<AttachmentCst>(1E3,0,Eigen::Vector3d(0,0,0)));
	// mSoftWorld->AddBody(p,cst_vec);
}
bool
SimWindow::
TimeStepping()
{
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
	glLineWidth(2.0);
	DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(100,0,0),Eigen::Vector3d(1,0,0));
	DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,100,0),Eigen::Vector3d(0,1,0));
	DrawLine(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,100),Eigen::Vector3d(0,0,1));
	glLineWidth(1.0);
	glPointSize(5.0);
	DrawPoint(random_target,Eigen::Vector3d(0,0,0));
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
	Eigen::VectorXd pos = mWorld->GetMusculoSkeletalSystem()->GetSkeleton()->getPositions();
	Eigen::VectorXd act = mWorld->GetMusculoSkeletalSystem()->GetActivationLevels();
	auto& skel = mWorld->GetMusculoSkeletalSystem()->GetSkeleton();
	Eigen::VectorXd random_pose_in_limits = pos;
	for(int i =0;i<skel->getNumDofs();i++)
		random_pose_in_limits[i] = dart::math::random(
										skel->getDof(i)->getPositionLowerLimit(),
										skel->getDof(i)->getPositionUpperLimit());

	

	random_target[0] = dart::math::random(-0.5,0.0);
	random_target[1] = dart::math::random(0.0,0.5);
	random_target[2] = dart::math::random(0.0,0.4);
	switch(key)
	{
		case 's' : TimeStepping();break;
		case '1' : act[1] += 0.1;break;
		case '2' : act[2] += 0.1;break;
		case '3' : act[3] += 0.1;break;
		case '4' : act[4] += 0.1;break;
		case '5' : act[5] += 0.1;break;
		case '6' : act[6] += 0.1;break;
		case '7' : act[7] += 0.1;break;
		case '8' : act[8] += 0.1;break;
		case '9' : act[9] += 0.1;break;
		case '0' : act[0] += 0.1;break;
		case 'r' : act.setZero();break;
		case 'b' : mWorld->GetController()->mTargetPositions = random_pose_in_limits;break;
		case 'c' : 
		mWorld->GetController()->AddIKTarget(std::make_pair(skel->getBodyNode("HandR"),Eigen::Vector3d(0,0,0)),random_target);
		mWorld->GetController()->SolveIK();
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
	while(!TimeStepping()){	}
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
}