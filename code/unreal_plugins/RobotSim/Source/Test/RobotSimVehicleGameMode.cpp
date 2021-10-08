// Copyright Epic Games, Inc. All Rights Reserved.

#include "RobotSimVehicleGameMode.h"
#include "RobotSimVehicleHud.h"
#include "SimpleVehicle/SimpleVehiclePawn.h"

ARobotSimVehicleGameMode::ARobotSimVehicleGameMode()
{
    DefaultPawnClass = ASimpleVehiclePawn::StaticClass();
    HUDClass = ARobotSimVehicleHud::StaticClass();
}
