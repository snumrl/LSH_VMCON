#ifndef __RECORD_H__
#define __RECORD_H__

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

	void Get(const dart::simulation::WorldPtr& rigid_world,
			 const std::shared_ptr<FEM::World>& soft_world);

	double			t;
	std::vector<Eigen::VectorXd> rigid_body_positions;
	std::vector<Eigen::VectorXd> rigid_body_velocities;
	Eigen::VectorXd soft_body_positions;

	void LoadFromFile(const std::string& path);
private:
	Record(){};
};

#endif