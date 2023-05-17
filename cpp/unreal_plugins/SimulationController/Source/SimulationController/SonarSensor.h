////------ BEGIN UE5 MIGRATION ------////
//// Uncomment this file when OpenBot is supported in UE5.
/*
//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <random>

#include <Engine/EngineBaseTypes.h>

class AActor;
class UBoxComponent;

class UTickEvent;

class SonarSensor 
{
public:
    SonarSensor(UBoxComponent* component);
    ~SonarSensor();

    // Measured sonar range
    float range_ = 0.0f;

private:
    // Updates Sonar measurements by shoting num_rays rays, randomly distributed in a pyramid,
    // parametrized by (range_min, range_max, horizontal_fov, vertical_fov), and return the
    // closest measured distance
    void postPhysicsPreRenderTickEventHandler(float delta_time, ELevelTick level_tick);

    AActor* parent_actor_ = nullptr;

    UBoxComponent* component_ = nullptr;

    UTickEvent* tick_event_ = nullptr;
    FDelegateHandle tick_event_handle_;

    // Random number generator
    std::minstd_rand random_gen_;
};
*/
////------ END UE5 MIGRATION ------////
