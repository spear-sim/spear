//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::align, std::unique_ptr

#include <Components/SceneCaptureComponent2D.h>
#include <Containers/Array.h>
#include <Containers/EnumAsByte.h>
#include <Engine/TextureRenderTarget2D.h> // ETextureRenderTargetFormat
#include <HAL/Platform.h>                 // SPCOMPONENTS_API, uint32
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
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCOMPONENTS_API USpSceneCaptureComponent2D : public USceneCaptureComponent2D
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
    template <typename T>
    static void UpdateArrayDataPtr(TArray<T>& array, void* data_ptr, int num_bytes) {

        SP_ASSERT(num_bytes % sizeof(T) == 0);
        int num_elements = num_bytes / sizeof(T);

        // We enforce the constraint that the array's existing data region must be at least as big as the
        // data_ptr region, because we want to guarantee that the array will not resize itself if the user
        // adds elements that would fit in the data_ptr region.

        // That being said, the data_ptr region may be smaller than the array's existing data region because
        // the array can reserve more space than was originally requested when calling array.Reserve(...).
        // Therefore, after calling UpdateArrayDataPtr(...), the user must be careful not to add more
        // elements to the array than would fit in the data_ptr region.

        SP_ASSERT(num_elements <= static_cast<int64_t>(array.Max()));
        SP_ASSERT(num_bytes <= array.GetAllocatedSize());

        // Check that data_ptr is sufficiently aligned for T.
        size_t num_bytes_size_t = num_bytes;
        T* data_ptr_aligned = static_cast<T*>(std::align(alignof(T), num_bytes_size_t, data_ptr, num_bytes_size_t));
        SP_ASSERT(data_ptr_aligned);
        SP_ASSERT(data_ptr == data_ptr_aligned);

        // Get pointer to array object, interpret as a pointer-to-T*.
        T** array_ptr = reinterpret_cast<T**>(&array);
        SP_ASSERT(array_ptr);

        // Check that the pointer to the array object, when interpreted as a pointer-to-T*, does indeed point
        // to the array's underlying data.
        SP_ASSERT(*array_ptr == array.GetData());

        // Update the array's underlying data pointer.
        *array_ptr = data_ptr_aligned;
        SP_ASSERT(data_ptr_aligned == array.GetData());
    };

    bool initialized_ = false;
    UMaterialInstanceDynamic* material_instance_dynamic_ = nullptr;
    std::unique_ptr<SharedMemoryRegion> shared_memory_region_ = nullptr;
    SpArraySharedMemoryView shared_memory_view_;

    // Scratchpad arrays for efficiently reading pixel data.
    TArray<FColor>        scratchpad_color_;
    TArray<FFloat16Color> scratchpad_float_16_color_;
    TArray<FLinearColor>  scratchpad_linear_color_;
};
