//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>

#include <Eigen/Core>
#include <memory>
#include <mujoco/mujoco.h>

#include "UrdfParser.h"

class UrdfMujocoControl
{
public:
    UrdfMujocoControl(std::string filename);
    ~UrdfMujocoControl();

    Eigen::VectorXf inverseDynamics(Eigen::VectorXf qpos);
    Eigen::VectorXf task_space_control(FTransform goal_pose, FTransform eef_pose,FVector velocity,FVector angular_velocity, Eigen::VectorXf qpos, Eigen::VectorXf qvel);

    mjModel* m = nullptr; // MuJoCo model
    mjData* d = nullptr;  // MuJoCo data
};
