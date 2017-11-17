#include "Juggling.h"

#include <iostream>


Juggling::
Juggling(const std::vector<int>& sequences,double T,double D)
	:mSequences(sequences),mT(T),mD(D),mCurrent(0)
{
	GenerateStates(sequences);
}
void
Juggling::
GenerateStates(const std::vector<int>& sequences,int n)
{
	int sn = sequences.size();
	int max_V = 0;
	int num_balls = 0;
	for(int i=0;i<sn;i++){
		num_balls += sequences[i];
		if(max_V<sequences[i])
			max_V = sequences[i];
	}
	num_balls /= sn;

	std::vector<int> lump_sequences;
	lump_sequences.resize(n);
	mStateSequences.resize(n);

	for(int i=0;i<n;i++){
		lump_sequences[i] = sequences[i%sn];
	}

	// mStateSequences[0].hand = LEFT_HAND;
	// mStateSequences[0].throw_hand = (mStateSequences[0].hand + lump_sequences[0])%2;
	// mStateSequences[0].V = lump_sequences[0];
	// mStateSequences[0].ball = 0;
	// mStateSequences[0].t_free = ((double)lump_sequences[0]-2*mD)*mT;

	for(int i=0;i<n;i++)
	{
		mStateSequences[i].hand = i%2;
		mStateSequences[i].throw_hand = (mStateSequences[i].hand + lump_sequences[i])%2;
		mStateSequences[i].V = lump_sequences[i];
		if(mStateSequences[i].ball == -1) //No before Throw
			mStateSequences[i].ball = i;
		mStateSequences[i].t_free = ((double)lump_sequences[i]-2*mD)*mT;

		int next_i = i + mStateSequences[i].V;
		if(next_i>=n)
			continue;
		mStateSequences[next_i].ball = mStateSequences[i].ball;
	}

	// for(int i=0;i<n;i++)
	// {
	// 	std::cout<<i<<std::endl;
	// 	std::cout<<(mStateSequences[i].hand==LEFT_HAND?"LEFT_HAND":"RIGHT_HAND")<<" -> ";
	// 	std::cout<<(mStateSequences[i].throw_hand==LEFT_HAND?"LEFT_HAND":"RIGHT_HAND")<<" V : "<<mStateSequences[i].V<<" of ball "<<mStateSequences[i].ball<<std::endl;
	// }
}
int 
Juggling::
GetBallNum()
{
	int sn = mSequences.size();
	int num_balls = 0;
	for(int i=0;i<sn;i++){
		num_balls += mSequences[i];
	}

	num_balls /= sn;
}