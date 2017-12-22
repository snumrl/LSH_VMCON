#ifndef __VMCON_SIM_WINDOW_H__
#define __VMCON_SIM_WINDOW_H__
#include "gui/gui.h"

class SimWindow : public GUI::GLUTWindow
{
public:
	SimWindow();

	double	activation_level;
	std::vector<FEM::LinearMuscleCstPtr> muscle_csts;
	FEM::WorldPtr 				mWorld;

	bool 						mIsPlay;
	bool 						mIsReplay;
	bool 						mIsPaused;
	bool 						mIsRotate;
	int 						mRecordFrame;
	double						mSimTime;

	bool TimeStepping();
protected:
	void Display() override;
	void Keyboard(unsigned char key,int x,int y) override;
	void Mouse(int button, int state, int x, int y) override;
	void Motion(int x, int y) override;
	void Reshape(int w, int h) override;
	void Timer(int value) override;
};

#endif