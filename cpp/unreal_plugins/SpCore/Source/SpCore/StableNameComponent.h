//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/ActorComponent.h>
#include <Containers/UnrealString.h> // FString
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS

#include "StableNameComponent.generated.h"

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCORE_API UStableNameComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UStableNameComponent();
    ~UStableNameComponent();

    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="Stable Name");
    FString StableName;

    #if WITH_EDITOR // defined in an auto-generated header
        // UActorComponent interface
        void OnComponentCreated() override;
        void PostLoad() override;

        // Interface for updating StableName if the component isn't spawned at runtime.
        void requestUpdate();
    #endif
};
