//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS

#include "StableNameComponent.generated.h"

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCORE_API UStableNameComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    UStableNameComponent();
    ~UStableNameComponent();

    #if WITH_EDITOR // defined in an auto-generated header
        // USceneComponent interface
        void OnComponentCreated() override;
        void PostLoad() override;
    #endif

    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "Stable Name");
    FString StableName;

    #if WITH_EDITOR // defined in an auto-generated header
        void requestUpdate();
    #endif
};
