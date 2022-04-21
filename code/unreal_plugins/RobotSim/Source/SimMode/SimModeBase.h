#pragma once

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

class AUrdfBotPawn;

/**
 * @brief
 *
 */
UCLASS()
class ROBOTSIM_API ASimModeBase : public AActor {
public:
    GENERATED_BODY()

public:
    /**
     * @brief Construct a new ASimModeBase object
     * Sets default values for this actor's properties
     */
    ASimModeBase();

    /**
     * @brief
     *
     */
    virtual void BeginPlay() override;

    /**
     * @brief
     *
     * @param EndPlayReason123
     */
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    /**
     * @brief 
     *
     * @param DeltaSeconds
     */
    virtual void Tick(float DeltaSeconds) override;

    /**
     * @brief Get the Global Ned Transform object
     *
     * @return const NedTransform&
     */
    const NedTransform& getGlobalNedTransform();
	
protected: // must overrides
    typedef RobotSim::RobotSimSettings RobotSimSettings;

    /**
     * @brief Get the Existing Vehicle Pawns object
     *
     * @param pawns
     */
    virtual void getExistingVehiclePawns(TArray<RobotBase*>& pawns) const;

protected: // optional overrides
    /**
     * @brief
     *
     */
    virtual void setupVehiclesAndCamera();

    /**
     * @brief
     *
     */
    virtual void setupInputBindings();
	
protected: // Utility methods for derived classes
    /**
     * @brief find a good location near ground 
     *
     * @param spawnPosition
     * @param spawnRotator
     * @param boxHalfSize
     */
    virtual void traceGround(FVector& spawnPosition,
                             FRotator& spawnRotator,
                             const FVector& boxHalfSize = FVector(0, 0, 0));

    /**
     * @brief Get the Nav Mesh object
     *
     * @return ARecastNavMesh*
     */
    virtual ARecastNavMesh* GetNavMesh();

    /**
     * @brief
     *
     * @param AgentRadius
     * @return true
     * @return false
     */
    virtual bool NavSystemRebuild(float AgentRadius);

    /**
     * @brief Find bounding box of all actors with architecture tag or furniture tag
     *
     * @param bScaleCeiling
     * @return FBox
     */
    virtual FBox GetWorldBoundingBox(bool bScaleCeiling = true);

    /**
     * @brief
     *
     * @return const RobotSim::RobotSimSettings&
     */
    virtual const RobotSim::RobotSimSettings& getSettings() const;

    /**
     * @brief Keyboard callback to test functions
     *
     */
    virtual void Test();

    /**
     * @brief Return all maps name in /Game/Maps
     *
     * @param MapList
     */
    virtual void GetAllMaps(TArray<FString>& MapList) const;

    /**
     * @brief Load a specific scene
     *
     * @param MapName
     */
    virtual void LoadMap(FString MapName);

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
    /**
     * @brief
     *
     */
    UPROPERTY()
    AVWLevelManager* level_manager_;
};
