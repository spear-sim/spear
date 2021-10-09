#pragma once

#include "Runtime/Engine/Public/EngineUtils.h"
#include "common_utils/EarthUtils.hpp"
#include "SimMode/SimModeBase.h"
#include "RobotBlueprintLib.h"
#include "RobotBase.h"
#include "SimpleVehicleApi.h"
#include "SimpleVehiclePawn.h"
#include "SimpleVehicleSimApi.h"
#include "SimModeSimpleVehicle.generated.h"

UCLASS()
class ROBOTSIM_API ASimModeSimpleVehicle : public ASimModeBase
{
    GENERATED_BODY()

public:
    ASimModeSimpleVehicle();

    virtual void BeginPlay() override;

    virtual void setupVehiclesAndCamera() override;
    virtual void
    getExistingVehiclePawns(TArray<RobotBase*>& pawns) const override;

    void cycleVisibleCameraForward();
    void cycleVisibleCameraBackward();
    void cycleVisibleCamera(bool forward);

    int camera_index_ = 0;
    std::vector<std::unique_ptr<SimpleVehicleSimApi>> vehicle_sim_apis_;

    TArray<APIPCamera*> cameras_;

    TArray<AActor*> spawned_actors_;
};