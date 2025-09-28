//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/PrimitiveComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString
#include <Engine/EngineTypes.h>      // EEndPlayReason
#include <HAL/Platform.h>            // SPUNREALTYPES_API

#include "SpUnrealTypes/SpUserInputComponent.h"

#include "SpBasicKeyboardControlComponent.generated.h"

struct FActorComponentTickFunction;

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPUNREALTYPES_API USpBasicKeyboardControlComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    USpBasicKeyboardControlComponent();
    ~USpBasicKeyboardControlComponent() override;

    // UActorComponent interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;

    // Specifies a component (whose path is AddRotationComponentPath) to add a rotation to.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString AddRotationComponentPath;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString AddRotationComponent;

    // Specifies a component (whose path is AddForceTargetComponentPath) to apply a force to, optionally mapping
    // the force vector into world-space from the component-space of another component (whose path is AddForceRotationComponentPath).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString AddForceTargetComponentPath;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString AddForceTargetComponent;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString AddForceRotationComponentPath;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString AddForceRotationComponent;

    // Required for custom debug keyboard commands.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    USpUserInputComponent* SpUserInputComponent = nullptr;

private:
    USceneComponent* add_rotation_component_ = nullptr;

    UPrimitiveComponent* add_force_target_component_ = nullptr;
    USceneComponent* add_force_rotation_component_ = nullptr;
};
