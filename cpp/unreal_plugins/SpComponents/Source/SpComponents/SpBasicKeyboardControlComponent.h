//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/PrimitiveComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString
#include <Engine/EngineTypes.h>      // EEndPlayReason
#include <HAL/Platform.h>            // SPCOMPONENTS_API

#include "SpComponents/SpUserInputComponent.h"

#include "SpBasicKeyboardControlComponent.generated.h"

struct FActorComponentTickFunction;

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCOMPONENTS_API USpBasicKeyboardControlComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    USpBasicKeyboardControlComponent();
    ~USpBasicKeyboardControlComponent();

    // UActorComponent interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString AddRotationComponentPath;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString AddRotationComponent;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString AddForceTargetComponentPath;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString AddForceTargetComponent;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    FString AddForceRotationComponentPath;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    FString AddForceRotationComponent;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="SPEAR")
    USpUserInputComponent* SpUserInputComponent = nullptr;

private:
    USceneComponent* add_rotation_component_ = nullptr;

    UPrimitiveComponent* add_force_target_component_ = nullptr;
    USceneComponent* add_force_rotation_component_ = nullptr;
};
