#ifndef __MUSCULO_SKELETAL_SYSTEM_H__
#define __MUSCULO_SKELETAL_SYSTEM_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"

#include "fem/fem.h"

typedef std::pair<dart::dynamics::BodyNode*,Eigen::Vector3d> AnchorPoint;
Eigen::Vector3d GetPoint(const AnchorPoint& ap);

struct Muscle
{
	void TransferForce(Eigen::Vector3d& f_origin,Eigen::Vector3d& f_insertion);
	void SetActivationLevel(double a);
	std::string 							name;
	FEM::MeshPtr							mesh;
	std::vector<AnchorPoint>				origin_way_points,insertion_way_points;
	double				activation_level;
	Eigen::Vector3d		origin_force,insertion_force;
	

	FEM::AttachmentCstPtr								origin,insertion;
	std::vector<FEM::LinearMuscleCstPtr>				muscle_csts;
	std::vector<FEM::CstPtr>							csts;
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Muscle(const Muscle& other) = delete;
	Muscle& operator=(const Muscle& other) = delete;
	std::shared_ptr<Muscle> Clone(const FEM::WorldPtr& soft_world,const dart::dynamics::SkeletonPtr& skeleton);
	static std::shared_ptr<Muscle> Create();
private:
	Muscle();

};


class MusculoSkeletalSystem
{
public:
	void AddMuscle(
		const std::string& name,
		const std::vector<AnchorPoint>& origin,
		const std::vector<AnchorPoint>& insertion,
		int origin_index,int insertion_index,
		const Eigen::Vector3d& fiber_direction,
		const FEM::MeshPtr& mesh);

	void Initialize(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world);

	void SetActivationLevels(const Eigen::VectorXd& a);
	void TransformAttachmentPoints();
	void ApplyForcesToSkeletons(const std::shared_ptr<FEM::World>& soft_world);

	int 									GetNumMuscles() 		{return mMuscles.size();}
	std::vector<std::shared_ptr<Muscle>>&	GetMuscles()			{return mMuscles;}
	dart::dynamics::SkeletonPtr&			GetSkeleton()			{return mSkeleton;}
	Eigen::VectorXd 						GetActivationLevels()	{return mActivationLevels;}

	MusculoSkeletalSystem(const MusculoSkeletalSystem& other) = delete;
	MusculoSkeletalSystem& operator=(const MusculoSkeletalSystem& other) = delete;
	std::shared_ptr<MusculoSkeletalSystem> Clone(const FEM::WorldPtr& soft_world,const dart::simulation::WorldPtr& rigid_world);
	static std::shared_ptr<MusculoSkeletalSystem> Create();
private:
	MusculoSkeletalSystem();
	//Muscles and Skeleton
	std::vector<std::shared_ptr<Muscle>>	mMuscles;
	dart::dynamics::SkeletonPtr 			mSkeleton;

	//Material Properties
	double	mTendonStiffness;
	double	mMuscleStiffness;
	double	mYoungsModulus;
	double	mPoissonRatio;

	Eigen::VectorXd							mActivationLevels;
};

void MakeMuscles(const std::string& path,std::shared_ptr<MusculoSkeletalSystem>& ms);
void MakeSkeleton(std::shared_ptr<MusculoSkeletalSystem>& ms);
#endif