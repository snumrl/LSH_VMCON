#ifndef __VMCON_SIM_WINDOW_H__
#define __VMCON_SIM_WINDOW_H__
#include "gui/gui.h"

#include "IntegratedWorld.h"

class SimWindow : public GUI::GLUTWindow
{
public:
	SimWindow();

	std::shared_ptr<IntegratedWorld> mWorld;

	bool 						mIsPlay;
	bool 						mIsReplay;
	bool 						mIsPaused;
	
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