//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Components/SceneCaptureComponent2D.h>
#include <Containers/EnumAsByte.h>
#include <Engine/TextureRenderTarget2D.h> // ETextureRenderTargetFormat
#include <HAL/Platform.h>                 // SPCOMPONENTS_API

#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/SpFuncArray.h"

#include "SpComponents/SpFuncArrayTypes.h"
#include "SpComponents/SpFuncComponent.h"

#include "SpSceneCaptureComponent2D.generated.h"

class UMaterial;
class UMaterialInstanceDynamic;

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCOMPONENTS_API USpSceneCaptureComponent2D : public USceneCaptureComponent2D
{
    GENERATED_BODY()
public:
    USpSceneCaptureComponent2D();
    ~USpSceneCaptureComponent2D();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void Initialize();
    UFUNCTION(CallInEditor, Category="SPEAR")
    void Terminate();
    UFUNCTION(CallInEditor, Category="SPEAR")
    bool IsInitialized();

    UPROPERTY(EditAnywhere, Category="SPEAR")
    int Width = 512;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    int Height = 512;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    int NumChannelsPerPixel = 4;

    UPROPERTY(EditAnywhere, Category="SPEAR")
    ESpFuncArrayDataType ChannelDataType = ESpFuncArrayDataType::UInt8;

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
    SpFuncSharedMemoryView shared_memory_view_;
};
