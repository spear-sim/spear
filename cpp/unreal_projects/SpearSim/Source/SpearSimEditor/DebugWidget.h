//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/UnrealString.h>
#include <CoreMinimal.h>
#include <GameFramework/Actor.h>

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
    void PrintDummyString();

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="Dummy string")
    FString DummyString;

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SpawnVehiclePawn();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SpawnUrdfRobotPawn();

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="URDF file")
    FString UrdfFile;
};
