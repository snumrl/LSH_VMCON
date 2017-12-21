#ifndef __RECORD_H__
#define __RECORD_H__
#include "fem/fem.h"
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
class MusculoSkeletalSystem;
class Record
{
public:
	Record(const Record& other) = delete;
	Record& operator=(const Record& other) = delete;
	std::shared_ptr<Record> Clone();
	static std::shared_ptr<Record> Create();

	void Get(const dart::simulation::WorldPtr& rigid_world,
			 const std::shared_ptr<FEM::World>& soft_world,
			 const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
			 Eigen::VectorXd& target);

	double			t;
	std::vector<Eigen::VectorXd> rigid_body_positions;
	std::vector<Eigen::VectorXd> rigid_body_velocities;
	Eigen::VectorXd soft_body_positions;
	Eigen::VectorXd activation_levels;
	Eigen::VectorXd target_positions;

	void LoadFromFile(const std::string& path);
private:
	Record(){};
};

#endif