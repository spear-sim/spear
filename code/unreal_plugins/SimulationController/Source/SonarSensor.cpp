#include "SonarSensor.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Components/PrimitiveComponent.h>
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

SonarSensor::SonarSensor(UPrimitiveComponent* component)
{
    ASSERT(component);

    new_object_parent_actor_ = component->GetWorld()->SpawnActor<AActor>();
    ASSERT(new_object_parent_actor_);

    range_min_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_PARAMETERS", "RANGE", "MIN"});
    ASSERT(range_min_ >= 0.0f);

    range_max_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_PARAMETERS", "RANGE", "MAX"});
    ASSERT(range_min_ <= range_max);

    horizontal_fov_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_PARAMETERS", "HORIZONTAL_FOV"});
    ASSERT(horizontal_fov_ >= 0.0f);

    vertical_fov_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_PARAMETERS", "VERTICAL_FOV"});
    ASSERT(vertical_fov_ >= 0.0f);

    noise_std_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_PARAMETERS", "NOISE_STD"});
    ASSERT(noise_std_ >= 0.0f);

    max_surface_reflection_angle_ = Config::getValue<float>({"SIMULATION_CONTROLLER", "SONAR_PARAMETERS", "MAX_REFLECTION_ANGLE"});
    ASSERT(max_surface_reflection_angle_ > 0.0);

    debug_ = Config::getValue<bool>({"SIMULATION_CONTROLLER", "SONAR_PARAMETERS", "DEBUG"});

    ASSERT(n_rays_ >= 1);
    rays_.clear();
    rays_.resize(n_rays_);

    trace_params_ = FCollisionQueryParams(FName(TEXT("SonarTrace")), true, new_object_parent_actor_);
    trace_params_.bTraceComplex = true;
    trace_params_.bReturnPhysicalMaterial = false;

    // Maximum sonar radius in horizontal and vertical direction
    max_rx_ = std::tanf(FMath::DegreesToRadians(horizontal_fov_ * 0.5f)) * range_max_ * new_object_parent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters;
    max_ry_ = std::tanf(FMath::DegreesToRadians(vertical_fov_ * 0.5f)) * range_max_ * new_object_parent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters;
}

SonarSensor::~SonarSensor()
{
    ASSERT(new_object_parent_actor_);
    new_object_parent_actor_->Destroy();
    new_object_parent_actor_ = nullptr;
}

void SonarSensor::update(float& range)
{
    const FTransform& actor_transform = new_object_parent_actor_->GetActorTransform();
    const FRotator& transform_rotator = actor_transform.Rotator();
    const FVector& sonar_location = new_object_parent_actor_->GetActorLocation();
    const FVector transform_x_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::X));
    const FVector transform_y_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::Y));
    const FVector transform_z_axis = transform_rotator.RotateVector(actor_transform.GetUnitAxis(EAxis::Z));
    float min_dist = range_max_;

    // Draw the sensing cone and the sonar rays
    if (debug_) {
        const FVector sensing_cone_vertex_1 = sonar_location + transform_rotator.RotateVector({range_max_ * new_object_parent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters, max_rx_, max_ry_});
        const FVector sensing_cone_vertex_2 = sonar_location + transform_rotator.RotateVector({range_max_ * new_object_parent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters, -max_rx_, max_ry_});
        const FVector sensing_cone_vertex_3 = sonar_location + transform_rotator.RotateVector({range_max_ * new_object_parent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters, -max_rx_, -max_ry_});
        const FVector sensing_cone_vertex_4 = sonar_location + transform_rotator.RotateVector({range_max_ * new_object_parent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters, max_rx_, -max_ry_});
        DrawDebugDirectionalArrow(new_object_parent_actor_->GetWorld(), sonar_location, sensing_cone_vertex_1, 0.15, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugDirectionalArrow(new_object_parent_actor_->GetWorld(), sonar_location, sensing_cone_vertex_2, 0.15, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugDirectionalArrow(new_object_parent_actor_->GetWorld(), sonar_location, sensing_cone_vertex_3, 0.15, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugDirectionalArrow(new_object_parent_actor_->GetWorld(), sonar_location, sensing_cone_vertex_4, 0.15, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugLine(new_object_parent_actor_->GetWorld(), sensing_cone_vertex_1, sensing_cone_vertex_2, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugLine(new_object_parent_actor_->GetWorld(), sensing_cone_vertex_2, sensing_cone_vertex_3, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugLine(new_object_parent_actor_->GetWorld(), sensing_cone_vertex_3, sensing_cone_vertex_4, FColor(255, 0, 0), false, 0.033, 0, 0.15);
        DrawDebugLine(new_object_parent_actor_->GetWorld(), sensing_cone_vertex_4, sensing_cone_vertex_1, FColor(255, 0, 0), false, 0.033, 0, 0.15);
    }

    FCriticalSection Mutex;
    new_object_parent_actor_->GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
    {
        ParallelFor(rays_.size(), [&](int idx) {
            FHitResult out_hit(ForceInit);
            float radius = std::uniform_real_distribution<float>()(random_gen_);
            float angle = std::uniform_real_distribution<float>(0.0f, 2 * PI)(random_gen_); // Uniform distibution of vales between 0 and 2*PI
            const FVector end_location = sonar_location + transform_rotator.RotateVector({range_max_ * new_object_parent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters, max_rx_ * radius * std::cosf(angle), max_ry_ * radius * std::sinf(angle)});
            const bool hit = new_object_parent_actor_->GetWorld()->LineTraceSingleByChannel(out_hit, sonar_location, end_location, ECollisionChannel::ECC_Visibility, trace_params_, FCollisionResponseParams::DefaultResponseParam);
            const TWeakObjectPtr<AActor> hit_actor = out_hit.Actor;
            FVector ray = (out_hit.ImpactPoint - sonar_location) / new_object_parent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters;

            if (hit && hit_actor.Get()) {

                // If the ange of hit surface normal and sonar ray is greater than 45°, then no reflection should be observed
                if (abs(FVector::DotProduct(out_hit.Normal, ray / ray.Size())) < std::cosf(FMath::DegreesToRadians(max_surface_reflection_angle_))) {
                    if (debug_) {
                        DrawDebugPoint(new_object_parent_actor_->GetWorld(), out_hit.ImpactPoint, 2, FColor(0, 0, 255), false, 0.033, 0);
                    }

                    rays_[idx].hit = false;
                }
                else { // A proper distance measurement can be made
                    if (debug_) {
                        DrawDebugPoint(new_object_parent_actor_->GetWorld(), out_hit.ImpactPoint, 3, FColor(0, 255, 0), false, 0.033, 0);
                    }

                    rays_[idx].hit = true;
                }
                if (debug_) {
                    DrawDebugLine(new_object_parent_actor_->GetWorld(), sonar_location, out_hit.ImpactPoint, FColor(200, 0, 200), false, 0.033, 0, 0.15);
                    DrawDebugDirectionalArrow(new_object_parent_actor_->GetWorld(), out_hit.ImpactPoint, out_hit.ImpactPoint + 5 * out_hit.Normal, 0.15, FColor(0, 188, 227), false, 0.033, 0, 0.15);
                }

                rays_[idx].azimuth_and_elevation = FMath::GetAzimuthAndElevation((end_location - sonar_location).GetSafeNormal() * range_max_, transform_x_axis, transform_y_axis, transform_z_axis);
                rays_[idx].distance = std::max(range_min_, out_hit.Distance / new_object_parent_actor_->GetWorld()->GetWorldSettings()->WorldToMeters);
            }
            else { // Out of range
                if (debug_) {
                    DrawDebugPoint(new_object_parent_actor_->GetWorld(), end_location, 2, FColor(255, 0, 0), false, 0.033, 0);
                }

                rays_[idx].hit = false;
                rays_[idx].azimuth_and_elevation = FVector2D::ZeroVector;
                rays_[idx].distance = range_max_;
            }

            if (rays_[idx].hit == true and rays_[idx].distance < min_dist) {
                min_dist = rays_[idx].distance; 
            }
        });
    }
    new_object_parent_actor_->GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

    range = min_dist + noise_std_ * std::uniform_real_distribution<float>()(random_gen_);
}
