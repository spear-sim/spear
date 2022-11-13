#include "SonarSensor.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Components/SceneComponent.h>
#include <DrawDebugHelpers.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <Math/UnrealMathUtility.h>
#include <PxScene.h>
#include <UObject/UObjectGlobals.h>

#include "Assert/Assert.h"
#include "Config.h"
#include "Serialize.h"
#include "TickEvent.h"

SonarSensor::SonarSensor(UPrimitiveComponent* primitive_component)
{
    ASSERT(primitive_component);
    primitive_component_ = primitive_component;
    new_object_parent_actor_ = primitive_component->GetWorld()->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    post_physics_pre_render_tick_event_ = NewObject<UTickEvent>(new_object_parent_actor_, TEXT("PostPhysicsPreRenderTickEvent"));
    ASSERT(post_physics_pre_render_tick_event_);
    post_physics_pre_render_tick_event_->RegisterComponent();
    post_physics_pre_render_tick_event_->initialize(ETickingGroup::TG_PostPhysics);
    post_physics_pre_render_tick_event_handle_ = post_physics_pre_render_tick_event_->delegate_.AddRaw(this, &SonarSensor::postPhysicsPreRenderTickEventHandler);

    range_min_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_SENSOR", "RANGE", "MIN"});
    ASSERT(range_min_ >= 0.0f);

    range_max_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_SENSOR", "RANGE", "MAX"});
    range = range_max_;
    ASSERT(range_min_ <= range_max_);

    horizontal_fov_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_SENSOR", "HORIZONTAL_FOV"});
    ASSERT(horizontal_fov_ >= 0.0f);

    vertical_fov_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_SENSOR", "VERTICAL_FOV"});
    ASSERT(vertical_fov_ >= 0.0f);

    noise_std_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_SENSOR", "NOISE_STD"});
    ASSERT(noise_std_ >= 0.0f);

    max_surface_reflection_angle_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_SENSOR", "MAX_REFLECTION_ANGLE"});
    ASSERT(max_surface_reflection_angle_ > 0.0);

    debug_ = Config::getValue<bool>({"SIMULATION_CONTROLLER", "SONAR_SENSOR", "DEBUG"});

    ASSERT(n_rays_ >= 1);
    rays_.clear();
    rays_.resize(n_rays_);

    trace_params_ = FCollisionQueryParams(FName(TEXT("SonarTrace")), true, new_object_parent_actor_);
    trace_params_.bTraceComplex = true;
    trace_params_.bReturnPhysicalMaterial = false;

    // Maximum sonar radius in horizontal and vertical direction
    max_rx_ = std::tanf(FMath::DegreesToRadians(horizontal_fov_ * 0.5f)) * range_max_ * primitive_component_->GetWorld()->GetWorldSettings()->WorldToMeters;
    max_ry_ = std::tanf(FMath::DegreesToRadians(vertical_fov_ * 0.5f)) * range_max_ * primitive_component_->GetWorld()->GetWorldSettings()->WorldToMeters;
}

SonarSensor::~SonarSensor()
{
    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;
}

void SonarSensor::postPhysicsPreRenderTickEventHandler(float delta_time, enum ELevelTick level_tick)
{
    const FTransform& actor_transform = new_object_parent_actor_->GetActorTransform();
    const FRotator& transform_rotator = actor_transform.Rotator();
    const FVector& sonar_location = new_object_parent_actor_->GetActorLocation();
    const FVector transform_x_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::X));
    const FVector transform_y_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::Y));
    const FVector transform_z_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::Z));
    range = range_max_;
    float min_dist = range_max_;

    FCriticalSection Mutex;
    primitive_component_->GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
    {
        for(const auto& ray : rays_) {
            FHitResult out_hit(ForceInit);
            float radius = std::uniform_real_distribution<float>()(random_gen_);
            float angle = std::uniform_real_distribution<float>(0.0f, 2 * PI)(random_gen_); // Uniform distibution of vales between 0 and 2*PI
            const FVector end_location = sonar_location + transform_rotator.RotateVector({range_max_ * primitive_component_->GetWorld()->GetWorldSettings()->WorldToMeters, max_rx_ * radius * std::cosf(angle), max_ry_ * radius * std::sinf(angle)});
            const bool hit = primitive_component_->GetWorld()->LineTraceSingleByChannel(out_hit, sonar_location, end_location, ECollisionChannel::ECC_Visibility, trace_params_, FCollisionResponseParams::DefaultResponseParam);
            const TWeakObjectPtr<AActor> hit_actor = out_hit.Actor;
            FVector ray_sonar = (out_hit.ImpactPoint - sonar_location) / primitive_component_->GetWorld()->GetWorldSettings()->WorldToMeters;

            if (hit && hit_actor.Get()) {

                // If the angle of hit surface normal and sonar ray is greater than 45°, then no reflection should be observed
                if (abs(FVector::DotProduct(out_hit.Normal, ray_sonar / ray_sonar.Size())) < std::cosf(FMath::DegreesToRadians(max_surface_reflection_angle_))) {
                    ray.hit = false;
                }
                else { // A proper distance measurement can be made
                    ray.hit = true;
                }
                ray.distance = std::max(range_min_, out_hit.Distance / primitive_component_->GetWorld()->GetWorldSettings()->WorldToMeters);
            }
            else { // Out of range
                ray.hit = false;
                ray.distance = range_max_;
            }

            if (ray.hit == true and ray.distance < min_dist) {
                min_dist = ray.distance;
            }
        }
    }
    primitive_component_->GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

    // Draw the sensing cone and the sonar rays
    if (debug_) {
        const FVector sensing_cone_vertex_1 = sonar_location + transform_rotator.RotateVector({range_max_ * primitive_component_->GetWorld()->GetWorldSettings()->WorldToMeters, max_rx_, max_ry_});
        const FVector sensing_cone_vertex_2 = sonar_location + transform_rotator.RotateVector({range_max_ * primitive_component_->GetWorld()->GetWorldSettings()->WorldToMeters, -max_rx_, max_ry_});
        const FVector sensing_cone_vertex_3 = sonar_location + transform_rotator.RotateVector({range_max_ * primitive_component_->GetWorld()->GetWorldSettings()->WorldToMeters, -max_rx_, -max_ry_});
        const FVector sensing_cone_vertex_4 = sonar_location + transform_rotator.RotateVector({range_max_ * primitive_component_->GetWorld()->GetWorldSettings()->WorldToMeters, max_rx_, -max_ry_});
        DrawDebugDirectionalArrow(primitive_component_->GetWorld(), sonar_location, sensing_cone_vertex_1, 0.15, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugDirectionalArrow(primitive_component_->GetWorld(), sonar_location, sensing_cone_vertex_2, 0.15, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugDirectionalArrow(primitive_component_->GetWorld(), sonar_location, sensing_cone_vertex_3, 0.15, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugDirectionalArrow(primitive_component_->GetWorld(), sonar_location, sensing_cone_vertex_4, 0.15, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugLine(primitive_component_->GetWorld(), sensing_cone_vertex_1, sensing_cone_vertex_2, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugLine(primitive_component_->GetWorld(), sensing_cone_vertex_2, sensing_cone_vertex_3, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugLine(primitive_component_->GetWorld(), sensing_cone_vertex_3, sensing_cone_vertex_4, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugLine(primitive_component_->GetWorld(), sensing_cone_vertex_4, sensing_cone_vertex_1, FColor(255, 0, 0), false, 0.033, 0, 0.15);

        for (const auto& ray : rays_) {
            DrawDebugLine(primitive_component_->GetWorld(), sonar_location, out_hit.ImpactPoint, FColor(200, 0, 200), false, 0.033, 0, 0.15);
            DrawDebugDirectionalArrow(primitive_component_->GetWorld(), out_hit.ImpactPoint, out_hit.ImpactPoint + 5 * out_hit.Normal, 0.15, FColor(0, 188, 227), false, 0.033, 0, 0.15);

            if (ray.hit) {
                DrawDebugPoint(primitive_component_->GetWorld(), out_hit.ImpactPoint, 5, FColor(0, 255, 0), false, 0.033, 0);
            }
            else {
                DrawDebugPoint(primitive_component_->GetWorld(), out_hit.ImpactPoint, 5, FColor(0, 0, 255), false, 0.033, 0);
            }
        }
    }

    range_ = min_dist + noise_std_ * std::uniform_real_distribution<float>()(random_gen_);
}
