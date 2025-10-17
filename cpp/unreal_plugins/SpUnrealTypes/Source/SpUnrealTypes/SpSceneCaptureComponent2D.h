//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Containers/EnumAsByte.h>
#include <Engine/TextureRenderTarget2D.h> // ETextureRenderTargetFormat
#include <HAL/Platform.h>                 // uint32
#include <Math/Color.h>                   // FColor, FLinearColor
#include <Math/Float16Color.h>
#include <TextureResource.h>              // FTextureRenderTargetResource

#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"

#include "SpSceneCaptureComponent2D.generated.h"

class UMaterial;
class UMaterialInstanceDynamic;

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Activation, AssetUserData, Collision, Cooking, LOD, Navigation, Physics, Rendering, Tags), meta=(BlueprintSpawnableComponent))
class USpSceneCaptureComponent2D : public USceneCaptureComponent2D
{
    GENERATED_BODY()
public:
    USpSceneCaptureComponent2D();
    ~USpSceneCaptureComponent2D() override;

    UFUNCTION(CallInEditor, Category="SPEAR")
    void Initialize();
    UFUNCTION(CallInEditor, Category="SPEAR")
    void Terminate();
    UFUNCTION(CallInEditor, Category="SPEAR")
    bool IsInitialized();

    UPROPERTY(EditAnywhere, Category="SPEAR")
    uint32 Width = 512;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    uint32 Height = 512;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    uint32 NumChannelsPerPixel = 4;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    ESpArrayDataType ChannelDataType = ESpArrayDataType::UInt8;

    // TEnumAsByte avoids: "Error: You cannot use the raw enum name as a type for member variables, instead use TEnumAsByte or a C++11 enum class with an explicit underlying type."
    UPROPERTY(EditAnywhere, Category="SPEAR")
    TEnumAsByte<ETextureRenderTargetFormat> TextureRenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8_SRGB;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    UMaterial* Material = nullptr;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bUseSharedMemory = true;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    bool bReadPixelData = true;

    UPROPERTY(VisibleAnywhere, Category="SPEAR")
    USpFuncComponent* SpFuncComponent = nullptr;

private:
    bool initialized_ = false;
    UMaterialInstanceDynamic* material_instance_dynamic_ = nullptr;
    std::unique_ptr<SharedMemoryRegion> shared_memory_region_ = nullptr;
    SpArraySharedMemoryView shared_memory_view_;

    // Scratchpad arrays for efficiently reading pixel data.
    TArray<FColor>        scratchpad_color_;
    TArray<FFloat16Color> scratchpad_float_16_color_;
    TArray<FLinearColor>  scratchpad_linear_color_;
};
