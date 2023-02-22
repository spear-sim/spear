//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <CoreMinimal.h>

#include <memory>

#include <mujoco/mujoco.h>

#include "UrdfParser.h"

class UrdfMujocoControl
{
public:
    UrdfMujocoControl(std::string filename);
    ~UrdfMujocoControl();

    void test();
    std::vector<float> get_qfrc_inverse(std::vector<float> qpos);

    mjModel* m = nullptr; // MuJoCo model
    mjData* d = nullptr;  // MuJoCo data
};
