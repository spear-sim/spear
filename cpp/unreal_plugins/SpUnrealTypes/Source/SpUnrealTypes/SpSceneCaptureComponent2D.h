//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Containers/EnumAsByte.h>
#include <Engine/EngineBaseTypes.h>       // ELevelTick
#include <Engine/EngineTypes.h>           // EEndPlayReason
#include <Engine/TextureRenderTarget2D.h> // ETextureRenderTargetFormat
#include <HAL/Platform.h>                 // int32, uint32
#include <Math/Color.h>                   // FColor, FLinearColor
#include <Math/Float16Color.h>
#include <SceneView.h>                    // FSceneViewFamily
#include <SceneViewExtension.h>           // FAutoRegister, FSceneViewExtensionBase, FSceneViewExtensions
#include <Templates/SharedPointer.h>      // TSharedPtr
#include <Templates/SharedPointerFwd.h>   // ESPMode
#include <TextureResource.h>              // FTextureRenderTargetResource
#include <UObject/ObjectMacros.h>         // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"

#include "SpSceneCaptureComponent2D.generated.h"

class UMaterial;
class UMaterialInstanceDynamic;
struct FActorComponentTickFunction;

class FSpSceneViewExtensionBase : public FSceneViewExtensionBase
{
public:
    FSpSceneViewExtensionBase(const FAutoRegister& auto_register, USpSceneCaptureComponent2D* component);

    void SetupViewFamily(FSceneViewFamily& in_view_family) override;
    void SetupView(FSceneViewFamily& in_view_family, FSceneView& in_view) override;
    void BeginRenderViewFamily(FSceneViewFamily& in_view_family) override;

protected:
    virtual void setupViewFamily(FSceneViewFamily& view_family) {};
    virtual void setupView(FSceneViewFamily& view_family, FSceneView& view) {};
    virtual void beginRenderViewFamily(FSceneViewFamily& view_family) {};

    USpSceneCaptureComponent2D* getComponent() { SP_ASSERT(component_); return component_; };

private:
    bool shouldHandleViewFamily(const FSceneViewFamily* view_family) const;
    bool shouldHandleView(const FSceneViewFamily* view_family, const FSceneView* view) const;

    USpSceneCaptureComponent2D* component_ = nullptr;
};

class FSpSceneViewExtension : public FSpSceneViewExtensionBase
{
public:
    FSpSceneViewExtension(const FAutoRegister& auto_register, USpSceneCaptureComponent2D* component);
protected:
    void setupView(FSceneViewFamily& view_family, FSceneView& view) override;
};

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Activation, AssetUserData, Collision, Cooking, LOD, Navigation, Physics, Rendering, Tags), meta=(BlueprintSpawnableComponent))
class USpSceneCaptureComponent2D : public USceneCaptureComponent2D
{
    GENERATED_BODY()
public:
    USpSceneCaptureComponent2D();
    ~USpSceneCaptureComponent2D() override;

    // UActorComponent interface
    void BeginPlay() override;
    void EndPlay(const EEndPlayReason::Type end_play_reason) override;

    UFUNCTION(CallInEditor, Category="SPEAR")
    void Initialize();
    UFUNCTION(CallInEditor, Category="SPEAR")
    void Terminate();
    UFUNCTION(Category="SPEAR")
    bool IsInitialized();

    // BlueprintReadWrite is incompatible with uint32 so we use int32

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    int32 Width = 512;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    int32 Height = 512;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    int32 NumChannelsPerPixel = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    ESpArrayDataType ChannelDataType = ESpArrayDataType::UInt8;

    // TEnumAsByte avoids: "Error: You cannot use the raw enum name as a type for member variables, instead use TEnumAsByte or a C++11 enum class with an explicit underlying type."
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TEnumAsByte<ETextureRenderTargetFormat> TextureRenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8_SRGB;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    UMaterial* Material = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bUseSharedMemory = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bReadPixelData = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bHideProxyComponentManagers = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TArray<FString> AllowedProxyComponentModalities;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bUseSceneViewExtension = false;

public:
    int32 getNumViewStates() { return ViewStates.Num(); }

private:
    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    bool bIsInitialized = false;

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;

    bool is_initialized_ = false;
    TSharedPtr<FSpSceneViewExtension, ESPMode::ThreadSafe> scene_view_extension_ = nullptr;
    UMaterialInstanceDynamic* material_instance_dynamic_ = nullptr;
    std::unique_ptr<SharedMemoryRegion> shared_memory_region_ = nullptr;
    SpArraySharedMemoryView shared_memory_view_;

    // Scratchpad arrays for efficiently reading pixel data.
    TArray<FColor>        scratchpad_color_;
    TArray<FFloat16Color> scratchpad_float_16_color_;
    TArray<FLinearColor>  scratchpad_linear_color_;
};
