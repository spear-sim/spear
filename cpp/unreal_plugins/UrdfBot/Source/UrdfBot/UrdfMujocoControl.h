//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>

#include <Eigen/Dense>
#include <memory>
#include <mujoco/mujoco.h>

#include "UrdfParser.h"

class UrdfMujocoControl
{
public:
    UrdfMujocoControl(std::string filename);
    ~UrdfMujocoControl();

    Eigen::VectorXf gravityCompensation(Eigen::VectorXf qpos);
    Eigen::VectorXf taskSpaceControl(FTransform goal_pose, FTransform eef_pose, FVector velocity, FVector angular_velocity, Eigen::VectorXf qpos, Eigen::VectorXf qvel);

    mjModel* m = nullptr;
    mjData* d = nullptr;
    int eef_id;
    double kp = 150;
    double kd = 25;

    static Eigen::Vector3d toEigen(FVector vector);
    static void printMatrix(std::string name, Eigen::MatrixXd mat);
};
