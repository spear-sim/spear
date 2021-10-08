// Copyright Epic Games, Inc. All Rights Reserved.
#pragma once
#include "GameFramework/HUD.h"
#include "RobotSimVehicleHud.generated.h"

UCLASS(config = Game)
class ARobotSimVehicleHud : public AHUD
{
    GENERATED_BODY()

public:
    ARobotSimVehicleHud();

    /** Font used to render the vehicle info */
    UPROPERTY()
    UFont* HUDFont;

    // Begin AHUD interface
    virtual void DrawHUD() override;
    // End AHUD interface
};
