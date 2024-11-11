//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/PrimitiveComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString

#include "SpComponents/SpUserInputComponent.h"

#include "SpDebugActionComponent.generated.h"

struct FActorComponentTickFunction;

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCOMPONENTS_API USpDebugActionComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    USpDebugActionComponent();
    ~USpDebugActionComponent();

    // UActorComponent interface
    void BeginPlay() override;

private:
    UPROPERTY(EditAnywhere, Category="SPEAR")
    FString AddRotationComponentPath;
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    FString AddRotationComponent;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    FString AddForceTargetComponentPath;
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    FString AddForceTargetComponent;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    FString AddForceRotationComponentPath;
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    FString AddForceRotationComponent;

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpUserInputComponent* SpUserInputComponent = nullptr;

    USceneComponent* add_rotation_component_ = nullptr;

    UPrimitiveComponent* add_force_target_component_ = nullptr;
    USceneComponent* add_force_rotation_component_ = nullptr;
};
