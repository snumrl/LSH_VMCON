#ifndef __RECORD_H__
#define __RECORD_H__

class MusculoSkeletalSystem;
class Controller;

namespace FEM
{
	class World;
};

class Record
{
public:
	Record(const Record& other) = delete;
	Record& operator=(const Record& other) = delete;
	std::shared_ptr<Record> Clone();
	static std::shared_ptr<Record> Create();

	void Set(const dart::simulation::WorldPtr& rigid_world,
			 const std::shared_ptr<FEM::World>& soft_world,
			 const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
			 const std::shared_ptr<Controller>& controller);
	void Get(const dart::simulation::WorldPtr& rigid_world,
			 const std::shared_ptr<FEM::World>& soft_world,
			 const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
			 const std::shared_ptr<Controller>& controller);

	double			t;
	std::vector<Eigen::VectorXd> rigid_body_positions;
	std::vector<Eigen::VectorXd> rigid_body_velocities;
	Eigen::VectorXd soft_body_positions;
	Eigen::VectorXd activation_levels;
	std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> muscle_forces;
private:
	Record(){};
};

#endif