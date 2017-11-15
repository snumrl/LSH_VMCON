#ifndef __FINITE_STATE_MACHINE_H__
#define __FINITE_STATE_MACHINE_H__
#include <vector>
#include <string>
#include <memory>

class FSM
{
public:
	void TriggerEvent(const std::string& name);
	const std::string& GetName(int idx);
	int GetState(const std::string& name);
	void AddEvent(const std::string& name, const std::string& from,const std::string& to);
	void AddState(const std::string& name);

	int GetCurrentState() {return mCurrentState;};
	FSM();
private:
	

	int 									mCurrentState;
	int 									mNumStates;
	std::vector<std::string>				mStates;
	std::vector<std::vector<std::string>> 	mEvents;
};

void MakeJugglingFSM(FSM& fsm);
#endif