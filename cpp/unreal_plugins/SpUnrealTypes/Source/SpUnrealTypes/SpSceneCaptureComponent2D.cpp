//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpSceneCaptureComponent2D.h"

#include <stdint.h> // uint64_t

#include <memory>  // std::align, std::make_unique
#include <utility> // std::move

#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Materials/Material.h>
#include <Materials/MaterialInstanceDynamic.h>
#include <Math/Color.h>              // FColor, FLinearColor
#include <Math/Float16Color.h>
#include <RHIDefinitions.h>          // ERangeCompressionMode
#include <RHITypes.h>                // FReadSurfaceDataFlags
#include <SceneView.h>               // FSceneViewFamily
#include <SceneManagement.h>         // FSceneViewStateInterface
#include <SceneViewExtension.h>      // FAutoRegister, FSceneViewExtensionBase, FSceneViewExtensions
#include <TextureResource.h>         // FTextureRenderTargetResource
#include <Templates/SharedPointer.h> // TSharedPtr
#include <UObject/UObjectGlobals.h>  // NewObject

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealArrayUpdateDataPtrScope.h"
#include "SpCore/UnrealUtils.h"

#include "SpUnrealTypes/SpPrimitiveProxyComponentManager.h"

FSpSceneViewExtensionBase::FSpSceneViewExtensionBase(const FAutoRegister& auto_register, USpSceneCaptureComponent2D* component) : FSceneViewExtensionBase(auto_register)
{
    SP_LOG_CURRENT_FUNCTION();

    SP_ASSERT(component);
    component_ = component;
}

void FSpSceneViewExtensionBase::SetupViewFamily(FSceneViewFamily& in_view_family)
{
    if (shouldHandleViewFamily(&in_view_family)) {
        setupViewFamily(in_view_family);
    }
}

void FSpSceneViewExtensionBase::SetupView(FSceneViewFamily& in_view_family, FSceneView& in_view)
{
    if (shouldHandleView(&in_view_family, &in_view)) {
        setupView(in_view_family, in_view);
    }
}

void FSpSceneViewExtensionBase::BeginRenderViewFamily(FSceneViewFamily& in_view_family)
{
    if (shouldHandleViewFamily(&in_view_family)) {
        beginRenderViewFamily(in_view_family);
    }
}

bool FSpSceneViewExtensionBase::shouldHandleViewFamily(const FSceneViewFamily* view_family) const
{
    SP_ASSERT(view_family);
    for (int i = 0; i < view_family->Views.Num(); i++) {
        const FSceneView* view = view_family->Views[i];
        if (shouldHandleView(view_family, view)) {
            return true;
        }
    }
    return false;
}

bool FSpSceneViewExtensionBase::shouldHandleView(const FSceneViewFamily* view_family, const FSceneView* view) const
{
    SP_ASSERT(component_);
    SP_ASSERT(view_family);
    SP_ASSERT(view);

    FSceneViewStateInterface* view_state = view->State;
    if (!view_state) {
        return false;
    }

    int32 view_state_key = view->State->GetViewKey();

    for (int32 i = 0; i < component_->getNumViewStates(); i++) {
        FSceneViewStateInterface* component_view_state = component_->GetViewState(i);
        if (component_view_state) {
            int32 component_view_state_key = component_view_state->GetViewKey();
            if (component_view_state_key > 0 && view_state_key > 0) { // compare keys if they're both valid
                if (component_view_state_key == view_state_key) {
                    return true;
                }
            } else { // otherwise compare pointers as a fallback to handle the case where one or both keys haven't been initialized yet
                if (component_view_state == view_state) {
                    return true;
                }
            }
        }
    }

    return false;
}

FSpSceneViewExtension::FSpSceneViewExtension(const FAutoRegister& auto_register, USpSceneCaptureComponent2D* component) : FSpSceneViewExtensionBase(auto_register, component)
{
    SP_LOG_CURRENT_FUNCTION();
}

void FSpSceneViewExtension::setupView(FSceneViewFamily& view_family, FSceneView& view)
{
    const TArray<FEngineShowFlagsSetting>& engine_show_flag_settings = getComponent()->GetShowFlagSettings();

    for (auto& engine_show_flag_setting : engine_show_flag_settings) {
        if (Unreal::toStdString(engine_show_flag_setting.ShowFlagName) == "LightingOnlyOverride" && engine_show_flag_setting.Enabled) {
            view.DiffuseOverrideParameter = FVector4f(GEngine->LightingOnlyBrightness.R, GEngine->LightingOnlyBrightness.G, GEngine->LightingOnlyBrightness.B, 0.0f);
            view.SpecularOverrideParameter = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
        }

        if (Unreal::toStdString(engine_show_flag_setting.ShowFlagName) == "Specular" && !engine_show_flag_setting.Enabled) {
            view.SpecularOverrideParameter = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
}

USpSceneCaptureComponent2D::USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();

    SpFuncComponent = UnrealUtils::createSceneComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
    SP_ASSERT(SpFuncComponent);

    // we want to be able to capture the scene even when the game is paused; note that we don't set
    // PrimaryActorTick.TickGroup because we don't want to interfere with when the base class component
    // gets ticked
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bTickEvenWhenPaused = true;

    // ensure that the underlying view state data is stable across frames so FSpSceneViewExtensionBase can
    // match view state data to this component
    bAlwaysPersistRenderingState = true;

    SetVisibility(false); // disable rendering to texture
}

USpSceneCaptureComponent2D::~USpSceneCaptureComponent2D()
{
    SP_LOG_CURRENT_FUNCTION();
}

void USpSceneCaptureComponent2D::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    USceneCaptureComponent2D::BeginPlay();
    bIsInitialized = false;
}

void USpSceneCaptureComponent2D::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

    Terminate();
    USceneCaptureComponent2D::EndPlay(end_play_reason);
}

void USpSceneCaptureComponent2D::Initialize()
{
    if (IsInitialized()) {
        return;
    }

    TextureTarget = NewObject<UTextureRenderTarget2D>(this);
    SP_ASSERT(TextureTarget);
    TextureTarget->RenderTargetFormat = TextureRenderTargetFormat;
    TextureTarget->ClearColor = FLinearColor(255, 0, 255); // bright pink
    TextureTarget->InitAutoFormat(Width, Height);
    bool clear_render_target = true;
    TextureTarget->UpdateResourceImmediate(clear_render_target);

    if (Material) {
        material_instance_dynamic_ = UMaterialInstanceDynamic::Create(Material, this);
        PostProcessSettings.AddBlendable(material_instance_dynamic_, 1.0f);
    }

    if (bHideProxyComponentManagers) {
        PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives; // use HiddenActors list
        std::vector<ASpPrimitiveProxyComponentManager*> primitive_proxy_component_managers = UnrealUtils::findActorsByType<ASpPrimitiveProxyComponentManager>(GetWorld());
        for (auto primitive_proxy_component_manager : primitive_proxy_component_managers) {
            HiddenActors.Add(primitive_proxy_component_manager);
        }
    }

    if (AllowedProxyComponentModalities.Num() > 0) {
        HiddenActors.Empty(); // clear HiddenActors because all actors will be hidden by default
        PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList; // use ShowOnlyActors list

        std::vector<std::string> allowed_modalities = Std::toVector<std::string>(
            Unreal::toStdVector(AllowedProxyComponentModalities) |
            std::views::transform([](auto& str) { return Unreal::toStdString(str); }));

        std::vector<ASpPrimitiveProxyComponentManager*> primitive_proxy_component_managers = Std::toVector<ASpPrimitiveProxyComponentManager*>(
            UnrealUtils::findActorsByType<ASpPrimitiveProxyComponentManager>(GetWorld()) |
            std::views::filter([&allowed_modalities](auto manager) { return Std::contains(allowed_modalities, manager->getModalityName()); }));

        for (auto primitive_proxy_component_manager : primitive_proxy_component_managers) {
            ShowOnlyActors.Add(primitive_proxy_component_manager);
        }
    }

    if (bUseSceneViewExtension) {
        scene_view_extension_ = FSceneViewExtensions::NewExtension<FSpSceneViewExtension>(this);
    }

    SetVisibility(true); // enable rendering to texture

    SpArrayDataType channel_data_type = Unreal::getEnumValueAs<SpArrayDataType, ESpArrayDataType>(ChannelDataType);
    uint64_t num_bytes = Height*Width*NumChannelsPerPixel*SpArrayDataTypeUtils::getSizeOf(channel_data_type);

    if (bUseSharedMemory) {
        SP_ASSERT(!shared_memory_region_);
        shared_memory_region_ = std::make_unique<SharedMemoryRegion>(num_bytes);
        SP_ASSERT(shared_memory_region_);
        shared_memory_view_ = SpArraySharedMemoryView(shared_memory_region_->getView(), "smem:sp_scene_capture_component_2d", SpArraySharedMemoryUsageFlags::ReturnValue);
        SpFuncComponent->registerSharedMemoryView(shared_memory_view_); // name needs to be unique per USpFuncComponent
    }

    if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::UInt8) {
        scratchpad_color_.Reserve(Height*Width);
        SP_ASSERT(Height*Width <= static_cast<int64_t>(scratchpad_color_.Max()));
        SP_ASSERT(num_bytes <= scratchpad_color_.GetAllocatedSize());
    } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float16) {
        scratchpad_float_16_color_.Reserve(Height*Width);
        SP_ASSERT(Height*Width <= static_cast<int64_t>(scratchpad_float_16_color_.Max()));
        SP_ASSERT(num_bytes <= scratchpad_float_16_color_.GetAllocatedSize());
    } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32) {
        scratchpad_linear_color_.Reserve(Height*Width);
        SP_ASSERT(Height*Width <= static_cast<int64_t>(scratchpad_linear_color_.Max()));
        SP_ASSERT(num_bytes <= scratchpad_linear_color_.GetAllocatedSize());
    } else {
        SP_ASSERT(false);
    }

    SpFuncComponent->registerFunc("read_pixels", [this, channel_data_type, num_bytes](SpFuncDataBundle& args) -> SpFuncDataBundle {

        SP_ASSERT(IsInitialized());
        SP_ASSERT(Height >= 0);
        SP_ASSERT(Width >= 0);
        SP_ASSERT(NumChannelsPerPixel == 4);

        SpPackedArray packed_array;
        packed_array.shape_ = {static_cast<uint64_t>(Height), static_cast<uint64_t>(Width), static_cast<uint64_t>(NumChannelsPerPixel)}; // explicit cast needed on Windows
        packed_array.data_type_ = channel_data_type;

        void* dest_ptr = nullptr;
        if (bUseSharedMemory) {
            packed_array.view_ = shared_memory_view_.data_;
            packed_array.data_source_ = SpArrayDataSource::Shared;
            packed_array.shared_memory_name_ = shared_memory_view_.name_;
            packed_array.shared_memory_usage_flags_ = shared_memory_view_.usage_flags_;
            dest_ptr = shared_memory_view_.data_;
        } else {
            Std::resizeUninitialized(packed_array.data_, num_bytes);
            packed_array.view_ = packed_array.data_.data();
            packed_array.data_source_ = SpArrayDataSource::Internal;
            dest_ptr = packed_array.data_.data();
        }
        
        SP_ASSERT(dest_ptr);

        if (bReadPixelData) {
            FTextureRenderTargetResource* texture_render_target_resource = TextureTarget->GameThread_GetRenderTargetResource();
            SP_ASSERT(texture_render_target_resource);

            // ReadPixels assumes 4 channels per pixel, 1 uint8 per channel
            if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::UInt8) {

                UnrealArrayUpdateDataPtrScope scope(scratchpad_color_, dest_ptr, num_bytes);
                bool success = texture_render_target_resource->ReadPixels(scratchpad_color_);
                SP_ASSERT(success);

            // ReadFloat16Pixels assumes 4 channels per pixel, 1 float16 per channel
            } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float16) {

                ERangeCompressionMode compression_mode = ERangeCompressionMode::RCM_MinMax;
                FReadSurfaceDataFlags read_surface_flags = FReadSurfaceDataFlags(compression_mode);
                read_surface_flags.SetLinearToGamma(false);

                UnrealArrayUpdateDataPtrScope scope(scratchpad_float_16_color_, dest_ptr, num_bytes);
                bool success = texture_render_target_resource->ReadFloat16Pixels(scratchpad_float_16_color_, read_surface_flags);
                SP_ASSERT(success);

            // ReadLinearColorPixels assumes 4 channels per pixel, 1 float32 per channel
            } else if (NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32) {
                SP_ASSERT(BOOST_OS_WINDOWS, "ERROR: NumChannelsPerPixel == 4 && channel_data_type == SpArrayDataType::Float32 is only supported on Windows.");

                ERangeCompressionMode compression_mode = ERangeCompressionMode::RCM_MinMax;
                FReadSurfaceDataFlags read_surface_flags = FReadSurfaceDataFlags(compression_mode);
                read_surface_flags.SetLinearToGamma(false);

                UnrealArrayUpdateDataPtrScope scope(scratchpad_linear_color_, dest_ptr, num_bytes);
                bool success = texture_render_target_resource->ReadLinearColorPixels(scratchpad_linear_color_, read_surface_flags);
                SP_ASSERT(success);

            } else {
                SP_ASSERT(false);
            }
        }

        SpFuncDataBundle return_values;
        Std::insert(return_values.packed_arrays_, "data", std::move(packed_array));

        return return_values;
    });

    is_initialized_ = true;
    bIsInitialized = true;
}

void USpSceneCaptureComponent2D::Terminate()
{
    if (!IsInitialized()) {
        return;
    }

    bIsInitialized = false;
    is_initialized_ = false;

    SpFuncComponent->unregisterFunc("read_pixels");

    if (shared_memory_region_) {
        SpFuncComponent->unregisterSharedMemoryView(shared_memory_view_);
        shared_memory_view_ = SpArraySharedMemoryView();
        shared_memory_region_ = nullptr;
    }

    SetVisibility(false); // disable rendering to texture
    bAlwaysPersistRenderingState = false;

    scene_view_extension_ = nullptr;

    ShowOnlyActors.Empty();
    HiddenActors.Empty();

    if (material_instance_dynamic_) {
        PostProcessSettings.RemoveBlendable(material_instance_dynamic_);
    }

    TextureTarget = nullptr;
}

bool USpSceneCaptureComponent2D::IsInitialized()
{
    return is_initialized_;
}
