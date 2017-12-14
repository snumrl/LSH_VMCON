#include "fem/fem.h"
#include "MusculoSkeletalSystem.h"
#include "Controller.h"
#include "Record.h"

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
	new_rec->activation_levels = activation_levels;
	new_rec->muscle_forces = muscle_forces;

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
Set(const dart::simulation::WorldPtr& rigid_world,
	const std::shared_ptr<FEM::World>& soft_world,
	const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
	const std::shared_ptr<Controller>& controller)
{
	t = rigid_world->getTime();
	rigid_body_positions.resize(rigid_world->getNumSkeletons());
	rigid_body_velocities.resize(rigid_world->getNumSkeletons());
	for(int i =0;i<rigid_world->getNumSkeletons();i++)
		rigid_body_positions[i] = rigid_world->getSkeleton(i)->getPositions();

	for(int i =0;i<rigid_world->getNumSkeletons();i++)
		rigid_body_velocities[i] = rigid_world->getSkeleton(i)->getVelocities();

	soft_body_positions = soft_world->GetPositions();
	activation_levels = musculo_skeletal_system->GetActivationLevels();
	for(auto& muscle : musculo_skeletal_system->GetMuscles())
		muscle_forces.push_back(std::make_pair(muscle->origin_force,muscle->insertion_force));
	target_positions = controller->GetTargetPositions();
}

void
Record::
Get(const dart::simulation::WorldPtr& rigid_world,
	const std::shared_ptr<FEM::World>& soft_world,
	const std::shared_ptr<MusculoSkeletalSystem>& musculo_skeletal_system,
	const std::shared_ptr<Controller>& controller)
{
	rigid_world->setTime(t);
	for(int i =0;i<rigid_world->getNumSkeletons();i++)
		rigid_world->getSkeleton(i)->setPositions(rigid_body_positions[i]);
	for(int i =0;i<rigid_world->getNumSkeletons();i++)
		rigid_world->getSkeleton(i)->setVelocities(rigid_body_velocities[i]);
	
	soft_world->SetTime(t);
	soft_world->SetPositions(soft_body_positions);
	int count = 0;
    for(auto& muscle : musculo_skeletal_system->GetMuscles())
    {
        muscle->SetActivationLevel(activation_levels[count]);
        muscle->origin_force = muscle_forces[count].first;
        muscle->insertion_force = muscle_forces[count].second;
        muscle->origin->SetP(GetPoint(muscle->origin_way_points[0]));
        muscle->insertion->SetP(GetPoint(muscle->insertion_way_points[0]));
        count++;
    }
    controller->SetTargetPositions(target_positions);
}

