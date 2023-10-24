//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/UnrealString.h> // FString
#include <GameFramework/Actor.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNTION, UPROPERTY

#include "DebugWidget.generated.h"

class FObjectInitializer;

UCLASS(Config=Spear, HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ADebugWidget : public AActor
{
    GENERATED_BODY()
public: 
    ADebugWidget(const FObjectInitializer& object_initializer);
    ~ADebugWidget();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void loadConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void saveConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void printDebugString();

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="Debug string")
    FString debug_string_;
};
