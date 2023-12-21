//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/UnrealString.h> // FString
#include <GameFramework/Actor.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "DebugWidget.generated.h"

UCLASS(Config=Spear, HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ADebugWidget : public AActor
{
    GENERATED_BODY()
public: 
    ADebugWidget();
    ~ADebugWidget();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void LoadConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SaveConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void PrintDebugString();

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="Debug string")
    FString DebugString;

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SpawnVehiclePawn();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SpawnUrdfRobotPawn();

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="URDF file")
    FString UrdfFile;
};
