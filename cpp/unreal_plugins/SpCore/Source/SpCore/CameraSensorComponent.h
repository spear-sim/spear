//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include <Camera/CameraComponent.h>
#include <Components/SceneComponent.h>
#include <Containers/UnrealString.h> // FString
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS

#include "SpCore/ArrayDesc.h"
#include "SpCore/Boost.h"
#include "SpCore/CppFuncComponent.h"

#include "CameraSensorComponent.generated.h"

class UCameraComponent;

struct RenderPassDesc
{
    USceneCaptureComponent2D* scene_capture_component_2d_ = nullptr;

    // strictly speaking, we could store width and height once for all render passes, but we store them here for simplicity
    int width_     = -1;
    int height_    = -1;
    int num_bytes_ = -1;

    // only used if SIMULATION_CONTROLLER.CAMERA_SENSOR.USE_SHARED_MEMORY is set to True
    std::string shared_memory_name_; // externally visible name
    std::string shared_memory_id_;   // ID used to manage the shared memory resource internally
    std::unique_ptr<SharedMemoryRegion> shared_memory_region_;
};

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup     = "SPEAR",
       HideCategories = (Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision),
       meta           = (BlueprintSpawnableComponent))
class SPCORE_API UCameraSensorComponent : public UCppFuncComponent
{
    GENERATED_BODY()
public:
    UCameraSensorComponent();
    ~UCameraSensorComponent();

    UFUNCTION(BlueprintCallable, Category = "SPEAR", meta = (DisplayName = "setup", ScriptName = "setup"))
    void setup(UCameraComponent* camera_component);

    UFUNCTION(BlueprintCallable, Category = "SPEAR", meta = (DisplayName = "get observation", ScriptName = "getObservation"))
    TArray<FColor> getObservation() const;
    UCameraComponent* camera_component_;
    std::map<std::string, RenderPassDesc> render_pass_descs_;
};
