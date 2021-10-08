// Copyright Epic Games, Inc. All Rights Reserved.

#include "RobotSimVehicleHud.h"
#include "RenderResource.h"
#include "Shader.h"
#include "Engine/Canvas.h"
#include "Engine/Font.h"
#include "CanvasItem.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/Engine.h"

#define LOCTEXT_NAMESPACE "VehicleHUD"

#ifndef HMD_MODULE_INCLUDED
#define HMD_MODULE_INCLUDED 0
#endif

ARobotSimVehicleHud::ARobotSimVehicleHud()
{
    static ConstructorHelpers::FObjectFinder<UFont> Font(
        TEXT("/Engine/EngineFonts/RobotoDistanceField"));
    HUDFont = Font.Object;
}

void ARobotSimVehicleHud::DrawHUD()
{
    Super::DrawHUD();

    // Calculate ratio from 720p
    const float HUDXRatio = Canvas->SizeX / 1280.f;
    const float HUDYRatio = Canvas->SizeY / 720.f;

    bool bWantHUD = true;
#if HMD_MODULE_INCLUDED
    bWantHUD = !GEngine->IsStereoscopic3D();
#endif // HMD_MODULE_INCLUDED
}

#undef LOCTEXT_NAMESPACE
