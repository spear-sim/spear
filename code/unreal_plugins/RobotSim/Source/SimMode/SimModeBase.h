#pragma once

#include "CoreMinimal.h"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "GameFramework/Actor.h"
#include "ParticleDefinitions.h"
#include "Kismet/KismetSystemLibrary.h"

#include "NavMesh/RecastNavMesh.h"
#include "common_utils/NavMeshUtil.hpp"

#include <string>
#include "common_utils/RobotSimSettings.hpp"
#include "NedTransform.h"
#include "RobotBase.h"
#include "SimModeBase.generated.h"

class AUrdfBotPawn;

UCLASS()
class ROBOTSIM_API ASimModeBase : public AActor
{
public:
    GENERATED_BODY()

public:
    // Sets default values for this actor's properties
    ASimModeBase();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

    const NedTransform& getGlobalNedTransform();

    virtual bool isUrdf()
    {
        return false;
    }

protected: // must overrides
    typedef RobotSim::RobotSimSettings RobotSimSettings;

    virtual void getExistingVehiclePawns(TArray<RobotBase*>& pawns) const;

protected: // optional overrides
    virtual void setupVehiclesAndCamera();
    virtual void setupInputBindings();

    virtual void traceGround(FVector& spawnPosition,
                             FRotator& spawnRotator,
                             const FVector& boxHalfSize = FVector(0, 0, 0));

    virtual ARecastNavMesh* GetNavMesh();
    virtual void NavSystemTest();
    virtual bool NavSystemRebuild(float AgentRadius);
    // find bounding box of all actors with architecture tag or furniture tag
    virtual FBox GetWorldBoundingBox(bool bScaleCeiling = true);
    ////called when SimMode should handle clock speed setting
    // virtual void setupClockSpeed();

protected: // Utility methods for derived classes
    virtual const RobotSim::RobotSimSettings& getSettings() const;
    // FRotator toFRotator(const RobotSimSettings::Rotation& rotation, const
    // FRotator& default_val);

protected:
    int record_tick_count;

    UPROPERTY() UClass* pip_camera_class;
    UPROPERTY() UParticleSystem* collision_display_template;

private:
    typedef common_utils::Utils Utils;
    typedef RobotSim::TTimePoint TTimePoint;
    typedef RobotSim::TTimeDelta TTimeDelta;

private:
    // assets loaded in constructor
    UPROPERTY() UClass* external_camera_class_;
    UPROPERTY() UClass* sky_sphere_class_;

    UPROPERTY() AActor* sky_sphere_;
    UPROPERTY() ADirectionalLight* sun_;
    ;
    TTimePoint tod_sim_clock_start_;
    TTimePoint tod_last_update_;
    std::time_t tod_start_time_;
    std::unique_ptr<NedTransform> global_ned_transform_;

    UPROPERTY()
    TArray<AActor*> spawned_actors_; // keep refs alive from Unreal GC

    bool lidar_checks_done_ = false;
    bool lidar_draw_debug_points_ = false;

private:
    void setupTimeOfDay();
};
