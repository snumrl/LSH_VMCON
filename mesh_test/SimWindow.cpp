#include "SimWindow.h"
#include <GL/glut.h>
using namespace GUI;
using namespace FEM;

extern std::string obj_path;
SimWindow::
SimWindow()
	:GLUTWindow(),mIsRotate(true),
	mIsPlay(false),mIsReplay(false),mIsPaused(false),mSimTime(0.0),mRecordFrame(0),activation_level(0)
{


	dart::math::seedRand();

	mWorld = FEM::World::Create(
		FEM::IntegrationMethod::PROJECTIVE_QUASI_STATIC,	//Integration Method
		// FEM::IntegrationMethod::QUASI_STATIC,	//Integration Method
		// FEM::IntegrationMethod::PROJECTIVE_DYNAMICS,	//Integration Method
		1.0/200.0,							//Time Step
		100,								//Max Iteration
		Eigen::Vector3d(0,-9.81,0),					//Gravity
		0.999								//Damping
		);

	double muscle_stiffness = 1E6;
	double poisson_ratio = 1;
	double Youngs_modulus =5E6;
	Eigen::Vector3d fiber_direction = Eigen::Vector3d::UnitY();
	Eigen::Affine3d T;
	T.setIdentity();
	T.linear() = 0.01*Eigen::Matrix3d::Identity();
	auto mesh = OBJMesh::Create(obj_path,T);
	std::vector<std::shared_ptr<Cst>> csts;
	const auto& tetrahedrons = mesh->GetTetrahedrons();
	const auto& vertices = mesh->GetVertices();
	int tet_index = 0;
	
	for(const auto& tet: tetrahedrons)
	{
		int i0,i1,i2,i3;
		Eigen::Vector3d p0,p1,p2,p3;

		i0 = tet[0];
		i1 = tet[1];
		i2 = tet[2];
		i3 = tet[3];

		p0 = vertices[i0];
		p1 = vertices[i1];
		p2 = vertices[i2];
		p3 = vertices[i3];

		Eigen::Matrix3d Dm;

		Dm.block<3,1>(0,0) = p1 -p0;
		Dm.block<3,1>(0,1) = p2 -p0;
		Dm.block<3,1>(0,2) = p3 -p0;

		//Peventing inversion
		if(Dm.determinant()<0)
		{
			i2 = tet[3];
			i3 = tet[2];
			
			p2 = vertices[i2];
			p3 = vertices[i3];

			Dm.block<3,1>(0,1) = p2-p0;
			Dm.block<3,1>(0,2) = p3-p0;
		}

		muscle_csts.push_back(LinearMuscleCst::Create("muscle_"+std::to_string(tet_index),
			muscle_stiffness,
			i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse(),
			fiber_direction));
		csts.push_back(muscle_csts.back());
		csts.push_back(CorotateFEMCst::Create("element_"+std::to_string(tet_index),
			Youngs_modulus,
			poisson_ratio,
			i0,i1,i2,i3,1.0/6.0*Dm.determinant(),Dm.inverse()));

		tet_index++;
	}
	double min_y=1000,max_y=-1000;
	std::vector<int> min_y_index,max_y_index;
	for(int i=0;i<vertices.size();i++)
	{
		if(min_y>vertices[i][1]-1E-6)
		{
			if(std::abs(min_y-vertices[i][1])<1E-6)
			{
				min_y_index.push_back(i);
			}
			else
			{
				min_y = vertices[i][1];
				min_y_index.clear();
			}
		}

		if(max_y<vertices[i][1]+1E-6)
		{
			if(std::abs(max_y-vertices[i][1])<1E-6)
			{
				max_y_index.push_back(i);
			}
			else
			{
				max_y = vertices[i][1];
				max_y_index.clear();
			}
		}
	}

	int last_index = vertices.size()-1;
	for(int i=0;i<min_y_index.size();i++)
	{
		csts.push_back(
			AttachmentCst::Create("origin",1E6,min_y_index[i],vertices[min_y_index[i]]));
	}
	for(int i=0;i<max_y_index.size();i++)
	{
		csts.push_back(
			AttachmentCst::Create("insertion",1E6,max_y_index[i],vertices[max_y_index[i]]));
	}

	Eigen::VectorXd v(vertices.size()*3);
	for(int i =0;i<vertices.size();i++)
		v.block<3,1>(i*3,0) = vertices[i];

	mWorld->AddBody(v,csts,1.0);
	mWorld->Initialize();

	mDisplayTimeout = 33;
}
bool
SimWindow::
TimeStepping()
{
	mWorld->TimeStepping();
	return true;
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

    GUI::DrawStringOnScreen(0.8,0.2,std::to_string(mWorld->GetTime()),true,Eigen::Vector3d(0,0,0));
	
	GUI::DrawWorld(mWorld);

	glutSwapBuffers();
}
void
SimWindow::
Keyboard(unsigned char key,int x,int y) 
{
	switch(key)
	{
		case '`' : mIsRotate = !mIsRotate;break;
		case '1' : activation_level += 0.1;break;
		case 'r' : activation_level = 0;break;
		case ' ' : mIsPlay = !mIsPlay; break;
		case 'p' : 
			if(!mIsReplay)
			{
				mIsReplay = true; mIsPlay = false; mIsPaused = true; mSimTime = mWorld->GetTime();
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
	if(activation_level>1.0)
		activation_level=1.0;

	for(auto m : muscle_csts)
		m->SetActivationLevel(activation_level);
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
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
}
