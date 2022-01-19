#pragma once

#include "Runtime/Engine/Public/EngineUtils.h"
#include "SimMode/SimModeBase.h"
#include "RobotBlueprintLib.h"
#include "RobotBase.h"
#include "UrdfBotPawn.h"
#include "RobotSimApi.h"
#include "UrdfBot/RobotApi.h"
#include "common_utils/EarthUtils.hpp"
#include "SimModeUrdfBot.generated.h"

UCLASS()
class ROBOTSIM_API ASimModeUrdfBot : public ASimModeBase
{
    GENERATED_BODY()

public:
    ASimModeUrdfBot();

    virtual void BeginPlay() override;

    virtual void setupVehiclesAndCamera() override;
    virtual void
    getExistingVehiclePawns(TArray<RobotBase*>& pawns) const override;

    void setupRobot(const FTransform& transform);

    void cycleVisibleCameraForward();
    void cycleVisibleCameraBackward();
    void cycleVisibleCamera(bool forward);
    void DestroyPawn();

    int camera_index_ = 0;

    std::vector<std::unique_ptr<RobotSimApi>> vehicle_sim_apis_;

    TArray<APIPCamera*> cameras_;

    TArray<AActor*> spawned_actors_;
    // count number of reset to avoid pawn name duplication
    int reset_count = 0;
};
