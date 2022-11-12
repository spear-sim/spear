#pragma once

#include <map>
#include <random>
#include <string>
#include <vector>

class AActor;
class USceneComponent;
class UTickEvent;

struct RayData {
        bool hit;
        FVector2D azimuth_and_elevation; // in [rad]
        float distance; // in [m]
    };

class SonarSensor 
{
public:
    SonarSensor(USceneComponent* component);
    SonarSensor(AActor* actor, float range_min, float range_max, float horizontal_fov, float vertical_fov, float noise_std, float max_surface_reflection_angle, const FVector& position_offset = FVector::ZeroVector, const FRotator& orientation_offset = FRotator::ZeroRotator, bool debug = false);
    ~SonarSensor();

    // Updates Sonar measurements by shoting n_rays rays, randomly distributed in a pyramid parametrized by (range_min, range_max, horizontal_fov, vertical_fov), and return the closest measured distance. 
    void postPhysicsPreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick);

    // Measured sonar range.
    float range = 0.0f;

private:

    AActor* new_object_parent_actor_ = nullptr;

    // Minimum sonar range in [m]
    float range_min_ = 0.02f;

    // Maximum sonar range in [m]
    float range_max_= 10.0f;

    // Sonar field of view in [deg]
    float horizontal_fov_ = 30.0f;
    float vertical_fov_ = 30.0f;

    // Number of rays used to emulate the sonar sensor
    unsigned int n_rays_ = 11;

    // Maximum sonar radius in horizontal and vertical direction (i.e. size of the base of the sensing pyramid)
    float max_rx_ = 0.0f;
    float max_ry_ = 0.0f;

    // Standard deviation of the measurement noise in [m]
    float noise_std_ = 0.0f;

    // Maximum angle of a surface, relative to the sensor's main axis, from which the emitted wave will no longer be reflected 
    float max_surface_reflection_angle_ = 45.0;

    FCollisionQueryParams trace_params_;

    std::minstd_rand random_gen_;
    std::vector<RayData> rays_;

    // Debug flag to enable debug markers 
    bool debug_ = false;

    UTickEvent* post_physics_pre_render_tick_event_ = nullptr;
    FDelegateHandle post_physics_pre_render_tick_event_handle_;
};
