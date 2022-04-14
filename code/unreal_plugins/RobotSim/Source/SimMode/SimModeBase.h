#pragma once

#include "CoreMinimal.h"
#include "Components/SkyLightComponent.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetSystemLibrary.h"
#include "NavMesh/RecastNavMesh.h"
#include "NedTransform.h"
#include "RobotBase.h"
#include "VWLevelManager.h"
#include "common_utils/NavMeshUtil.hpp"
#include "common_utils/RobotSimSettings.hpp"

#include "SimModeBase.generated.h"

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
	
protected: // must overrides
    typedef RobotSim::RobotSimSettings RobotSimSettings;

    virtual void getExistingVehiclePawns(TArray<RobotBase*>& pawns) const;

protected: // optional overrides
    virtual void setupVehiclesAndCamera();
    virtual void setupInputBindings();

protected: // Utility methods for derived classes
    virtual const RobotSim::RobotSimSettings& getSettings() const;
    //try to move the spawn location close to ground
    virtual void traceGround(FVector& spawnPosition,
                             FRotator& spawnRotator,
                             const FVector& boxHalfSize = FVector(0, 0, 0));
	// return current NavMesh actor
    virtual ARecastNavMesh* GetNavMesh();
    virtual bool NavSystemRebuild(float AgentRadius);
    // find bounding box of all actors with architecture tag or furniture tag
    virtual FBox GetWorldBoundingBox(bool bScaleCeiling = true);
    // return all maps name in /Game/Maps
    virtual void GetAllMaps(TArray<FString>& MapList) const;
    // load specific scene
    virtual void LoadMap(FString MapName);

    // test only - keyboard callback to test functions
    virtual void Test();

protected:
    UPROPERTY()
    UClass* pip_camera_class;
    UPROPERTY()
    UParticleSystem* collision_display_template;

private:
    typedef common_utils::Utils Utils;
    typedef RobotSim::TTimePoint TTimePoint;
    typedef RobotSim::TTimeDelta TTimeDelta;

private:
    // assets loaded in constructor
    UPROPERTY()
    UClass* external_camera_class_;
    UPROPERTY()
    UClass* sky_sphere_class_;

    UPROPERTY()
    AActor* sky_sphere_;
    std::unique_ptr<NedTransform> global_ned_transform_;

    UPROPERTY()
    TArray<AActor*> spawned_actors_; // keep refs alive from Unreal GC

private:
    UPROPERTY()
    AVWLevelManager* levelManager;
};
