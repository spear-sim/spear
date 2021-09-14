#pragma once

#include "CoreMinimal.h"

#include "RobotBlueprintLib.h"

#include "iostream"

#include "rbdl/rbdl.h"
#include "rbdl/rbdl_utils.h"

#include "rbdl/addons/urdfreader/urdfreader.h"

typedef Eigen::Matrix<double, 4, 4> Matrix44;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class InverseKinematicComponent
{
public:
	InverseKinematicComponent();
	//load model from urdf file path
	void ImportModel(std::string FilePath, float ScaleFactor = 1.0f);
	//set end effector link
	void SetEndEffector(FString endEffectorName);
	//find target qpos based on target eef_pose, current qpos
	bool CalculateIK(TMap<FString, float> Qinit, FString endEffectorName, FVector targetPos, FQuat targetOri, TMap<FString, float>& resultMap);
	//find given body pose based on current qpos
	Matrix44 ForwardKinematic(VectorNd Q, int body_id);

private:
	//calculate IK with current torso pose preferred torso_lift_joint only changed if required
	bool CalculateIKWithControlledTorso(const Math::VectorNd &Qinit, Math::VectorNd &Qres, int body_id, Vector3d targetPos, Matrix3d targetOri);

private:
	Model* model = new Model();
	TArray<int> armJointIds;
	std::string eefName_;
	int eefId_;
	float invScaleFactor_ = 1.0f;
};