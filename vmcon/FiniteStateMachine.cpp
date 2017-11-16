#include "FiniteStateMachine.h"
#include <iostream>
#include <cassert>
void
FSM::
TriggerEvent(const std::string& name)
{
	for(int i =0;i<mNumStates;i++)
		if(!name.compare(mEvents[mCurrentState][i])){
			mCurrentState = i;
			return;
		}

	std::cout<<"No Event name : "<<name<<std::endl;
}
const std::string&
FSM::
GetName(int idx)
{
	assert( idx>=0 && idx<mNumStates);
	return mStates[idx];
}
int
FSM::
GetState(const std::string& name)
{
	for(int i=0;i<mNumStates;i++)
		if(!name.compare(mStates[i]))
			return i;

	std::cout<<"No State name : "<<name<<std::endl;
	return -1;
}

void
FSM::
AddEvent(const std::string& name, const std::string& from,const std::string& to)
{
	int from_ = GetState(from);
	int to_ = GetState(to);

	mEvents[from_][to_] = name;
}
void
FSM::
AddState(const std::string& name)
{
	mStates.push_back(name);
	mNumStates++;
	mEvents.resize(mNumStates);
	for(int i=0;i<mNumStates;i++)
		mEvents[i].resize(mNumStates);
}
FSM::
FSM()
	:mNumStates(0),mCurrentState(0)
{

}

void
MakeJugglingFSM(FSM& fsm)
{
	fsm.AddState("SWING");
	fsm.AddState("CATCH");
	fsm.AddEvent("catch_success","CATCH","SWING");
	fsm.AddEvent("swing_end","SWING","CATCH");
}