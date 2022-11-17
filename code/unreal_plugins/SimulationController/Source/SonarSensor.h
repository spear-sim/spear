#pragma once

#include <random>

#include <Engine/EngineBaseTypes.h>
#include <Math/Vector.h>

class AActor;
class UBoxComponent;
class UTickEvent;

class SonarSensor 
{
public:
    SonarSensor(UBoxComponent* component);
    ~SonarSensor();

    // Updates Sonar measurements by shoting num_rays rays, randomly distributed in a pyramid,
    // parametrized by (range_min, range_max, horizontal_fov, vertical_fov), and return the closest measured distance. 
    void postPhysicsPreRenderTickEventHandler(float delta_time, ELevelTick level_tick);

    // Measured sonar range
    float range_ = 0.0f;

private:
    AActor* new_object_parent_actor_ = nullptr;
    UBoxComponent* component_ = nullptr;

    UTickEvent* tick_event_ = nullptr;
    FDelegateHandle tick_event_handle_;

    // Random number generator
    std::minstd_rand random_gen_;
};
