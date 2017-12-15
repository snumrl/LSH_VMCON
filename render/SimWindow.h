#ifndef __VMCON_SIM_WINDOW_H__
#define __VMCON_SIM_WINDOW_H__
#include "gui/gui.h"

#include "World.h"
#include "Record.h"

class SimWindow : public GUI::GLUTWindow
{
public:
	SimWindow();

	std::shared_ptr<IntegratedWorld> mWorld;
	std::vector<std::shared_ptr<Record>>		mRecords;
	int 						mFrame;
	bool 						mIsPlay;
	bool 						mIsRotate;
	bool 						mIsDrag;

	
	void LoadFromFolder(const std::string& path);
protected:
	void Display() override;
	void Keyboard(unsigned char key,int x,int y) override;
	void Mouse(int button, int state, int x, int y) override;
	void Motion(int x, int y) override;
	void Reshape(int w, int h) override;
	void Timer(int value) override;
};

#endif