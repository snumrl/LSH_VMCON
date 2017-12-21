#include <fstream>

#include "Record.h"
#include "../vmcon/MusculoSkeletalSystem.h"
#include <iostream>

using namespace dart::dynamics;
using namespace dart::simulation;
std::shared_ptr<Record>
Record::
Clone()
{
	auto new_rec = Create();

	new_rec->t = t;
	new_rec->rigid_body_positions = rigid_body_positions;
	new_rec->rigid_body_velocities = rigid_body_velocities;
	new_rec->soft_body_positions = soft_body_positions;

	return new_rec;
}
std::shared_ptr<Record>
Record::
Create()
{
	auto rec = new Record();
	return std::shared_ptr<Record>(rec);
}

void
Record::
Get(const dart::simulation::WorldPtr& rigid_world,
	const std::shared_ptr<FEM::World>& soft_world,
	const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
	Eigen::VectorXd& target)
{
	rigid_world->setTime(t);
	for(int i =0;i<rigid_world->getNumSkeletons();i++)
		rigid_world->getSkeleton(i)->setPositions(rigid_body_positions[i]);
	for(int i =0;i<rigid_world->getNumSkeletons();i++){
		rigid_world->getSkeleton(i)->setVelocities(rigid_body_velocities[i]);
	}
	soft_world->SetTime(t);
	soft_world->SetPositions(soft_body_positions);
	int count = 0;
    for(auto& muscle : musculo_skeletal_system->GetMuscles())
    {
        muscle->SetActivationLevel(activation_levels[count]);
        muscle->origin->SetP(GetPoint(muscle->origin_way_points[0]));
        muscle->insertion->SetP(GetPoint(muscle->insertion_way_points[0]));
        count++;
    }
    target = target_positions;
}

void
Record::
LoadFromFile(const std::string& path)
{
	std::ifstream ifs(path);
	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<path<<std::endl;
		return;
	}
	std::string str;
	std::string index;
	std::stringstream ss;

	while(!ifs.eof())
	{
		str.clear();
		index.clear();
		ss.clear();

		std::getline(ifs,str);
		ss.str(str);	
		ss>>index;

		Eigen::VectorXd eigen_vec;
		std::vector<double> vec;
		double val;
		if(!index.compare("soft"))
		{
			while(!ss.eof())
			{
				ss>>val;
				vec.push_back(val);
			}
			eigen_vec.resize(vec.size());
			for(int i=0;i<vec.size();i++)
			{
				eigen_vec[i] = vec[i];
			}
			soft_body_positions = eigen_vec;
		}
		else if(!index.compare("rpos"))
		{
			while(!ss.eof())
			{
				ss>>val;
				vec.push_back(val);
			}
			eigen_vec.resize(vec.size());
			for(int i=0;i<vec.size();i++)
			{
				eigen_vec[i] = vec[i];
			}
			rigid_body_positions.push_back(eigen_vec);
		}
		else if(!index.compare("rvel"))
		{
			while(!ss.eof())
			{
				ss>>val;
				vec.push_back(val);
			}
			eigen_vec.resize(vec.size());
			for(int i=0;i<vec.size();i++)
			{
				eigen_vec[i] = vec[i];
			}
			rigid_body_velocities.push_back(eigen_vec);

		}
		else if(!index.compare("time"))
		{
			ss>>val;
			t = val;
		}
		else if(!index.compare("act"))
		{
			while(!ss.eof())
			{
				ss>>val;
				vec.push_back(val);
			}
			eigen_vec.resize(vec.size());
			for(int i=0;i<vec.size();i++)
			{
				eigen_vec[i] = vec[i];
			}
			activation_levels =eigen_vec;
		}
		else if(!index.compare("target"))
		{
			while(!ss.eof())
			{
				ss>>val;
				vec.push_back(val);
			}
			eigen_vec.resize(vec.size());
			for(int i=0;i<vec.size();i++)
			{
				eigen_vec[i] = vec[i];
			}
			target_positions = eigen_vec;
		}
		
	}
	ifs.close();
}