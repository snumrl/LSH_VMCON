#ifndef __JUGGLING_H__
#define __JUGGLING_H__
#include <vector>
#include <string>
struct JugglingState
{
	int 	hand;
	int 	throw_hand;

	int 	V;
	int 	ball;

	double 	t_free;
	JugglingState():ball(-1){};
};

struct Juggling
{
public:
	static const int LEFT_HAND = 0;
	static const int RIGHT_HAND = 1;

	std::vector<JugglingState> 	mStateSequences;
	std::vector<int>			mSequences;
	int 						mCurrent;

	double						mT;			//time per sequence.
	double						mD;			//time per throw.

	Juggling(const std::vector<int>& sequences = {3,3,3},double T = 0.3,double D = 0.7);
	int GetBallNum();
	void GenerateStates(const std::vector<int>& sequences,int n = 40);	
};

#endif