#include "Juggling.h"
#include <iostream>
JugglingInfo::
JugglingInfo(const std::vector<int>& V_sequences,int ball_size)
	:V(V_sequences),left("HandL"),right("HandR"),count(0),T(0.3),D(0.7)
{
	//Generate ball index.
	ball_index.resize(V.size(),-1);

	
	for(int i = 0;i<ball_size;i++)
	{
		int count_i = i;
		bool is_full = true;
		for(int j =i;j<V.size();j++)
		{
			if(ball_index[j]==-1)
			{
				count_i = j;
				is_full = false;
				break;
			}
		}

		if(is_full)
			break;
		
		while(ball_index[count_i]==-1)
		{
			ball_index[count_i] = i;
			if(V[count_i]==0)
				break;
			count_i += V[count_i];
			if(count_i>=V.size())
				break;
		}
	}

	
	

	std::cout<<"ball : ";
	for(auto b : ball_index)
		std::cout<<b<<" ";
	std::cout<<std::endl;


	// for(int i = 0;i<V.size();i++)
	// 	V[i] = i+3;
	// 	count = i;
	// 	std::cout<<From()<<"->"<<To()<<" : "<<GetV()<<std::endl;
	// }
	count = 0;
}
void
JugglingInfo::
CountPlusPlus()
{
	count++;
	// std::cout<<count<<" : "<<From()<<"->"<<To()<<" V("<<GetV()<<")"<<std::endl;
}
void
JugglingInfo::
CountMinusMinus()
{
	count--;
	// std::cout<<count<<" : "<<From()<<"->"<<To()<<" V("<<GetV()<<")"<<std::endl;
}

Eigen::Vector3d
JugglingInfo::
GetTargetVelocity(const Eigen::Vector3d& from,const Eigen::Vector3d& to)
{	
	Eigen::Vector3d diff = to-from;
	// double t_free_vec[] = {0.3,0.5,0.7,0.9,1.1,1.3,1.5,1.7,1.9,2.1};
	// double t_free = t_free_vec[count];
	double t_free = GetT_free();
	
	double dh = from[1] - to[1];
	double v_0y = 0.5*9.81*t_free - dh/t_free;

	double v_0x = diff[0]/t_free;
	double v_0z = diff[2]/t_free;

	return Eigen::Vector3d(v_0x,v_0y,v_0z);
}

std::string 
JugglingInfo::
From()
{
	if(count%2 == 0 )
		return right;
	else
		return left;
}
std::string 
JugglingInfo::
To()
{
	int next_count = count+GetV();
	
	if(next_count%2==0)
		return right;
	else
		return left;
}
