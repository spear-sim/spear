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
    void spawnUrdfRobotPawn();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void loadConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void saveConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void printDummyString();

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="URDF File")
    FString urdf_file_;

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="Dummy String")
    FString dummy_string_;
};
