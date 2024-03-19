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

    // If we're in the editor, then set StableName when the component is created or when the editor starts.
    #if WITH_EDITOR // defined in an auto-generated header
        void OnComponentCreated() override;
        void PostLoad() override;
    #endif

    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "Stable Name");
    FString StableName;

    // If we're in the editor, provide a public function to set StableName. This is useful, e.g., when the
    // owning actor is renamed or moved in the World Outliner.
#if WITH_EDITOR // defined in an auto-generated header
        void update();
    #endif
};
