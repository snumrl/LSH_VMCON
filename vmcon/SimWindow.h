#ifndef __VMCON_SIM_WINDOW_H__
#define __VMCON_SIM_WINDOW_H__
#include "gui/gui.h"
#include "fem/fem.h"
#include "MusculoSkeletalSystem.h"
namespace VMCON
{
class SimWindow : public GUI::GLUTWindow
{
public:
	SimWindow();

	std::shared_ptr<FEM::World> mSoftWorld;
	dart::simulation::WorldPtr mRigidWorld;
	std::shared_ptr<MusculoSkeletalSystem> mMusculoSkeletalSystem;

	void TimeStepping();
protected:
	void Display() override;
	void Keyboard(unsigned char key,int x,int y) override;
	void Mouse(int button, int state, int x, int y) override;
	void Motion(int x, int y) override;
	void Reshape(int w, int h) override;
	void Timer(int value) override;
};

};
#endif