//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Components/SceneCaptureComponent2D.h>
#include <Engine/EngineTypes.h> // EEndPlayReason

#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/SpFuncArray.h"

#include "SpComponents/SpFuncComponent.h"

#include "SpSceneCaptureComponent2D.generated.h"

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCOMPONENTS_API USpSceneCaptureComponent2D : public USceneCaptureComponent2D
{
    GENERATED_BODY()
public:
    USpSceneCaptureComponent2D();
    ~USpSceneCaptureComponent2D();

    // USceneCaptureComponent2D interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;

private:
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;

    std::unique_ptr<SharedMemoryRegion> shared_memory_region_ = nullptr;
    SpFuncSharedMemoryView shared_memory_view_;
};
