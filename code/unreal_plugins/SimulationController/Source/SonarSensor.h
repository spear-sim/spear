#pragma once

#include <map>
#include <random>
#include <string>
#include <vector>

class AActor;
class USkeletalMeshComponent;
class UTickEvent;

class SonarSensor 
{
public:
    SonarSensor(USkeletalMeshComponent* primitive_component);
    ~SonarSensor();

    // Updates Sonar measurements by shoting n_rays rays, randomly distributed in a pyramid parametrized by (range_min, range_max, horizontal_fov, vertical_fov), and return the closest measured distance. 
    void postPhysicsPreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick);

    // Measured sonar range.
    float range_ = 0.0f;

private:

    AActor* new_object_parent_actor_ = nullptr;

    USkeletalMeshComponent* primitive_component_ = nullptr;

    UTickEvent* post_physics_pre_render_tick_event_ = nullptr;
    FDelegateHandle post_physics_pre_render_tick_event_handle_;

    std::minstd_rand random_gen_;
};
