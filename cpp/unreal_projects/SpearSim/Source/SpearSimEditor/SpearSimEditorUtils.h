//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/UnrealString.h>
#include <CoreMinimal.h>
#include <GameFramework/Actor.h>

#include "SpearSimEditorUtils.generated.h"

class FObjectInitializer;

UCLASS(Config=Spear, HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ASpearSimEditorUtils : public AActor
{
    GENERATED_BODY()
public: 
    ASpearSimEditorUtils(const FObjectInitializer& object_initializer);
    ~ASpearSimEditorUtils();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void loadConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void saveConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void printDummyString();

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="Dummy String")
    FString dummy_string_;
};
