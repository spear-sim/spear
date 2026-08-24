//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <EngineModule.h>                    // GetRendererModule
#include <Kismet/BlueprintFunctionLibrary.h>
#include <RHICommandList.h>                  // FRHICommandListImmediate
#include <RHIFeatureLevel.h>                 // ERHIFeatureLevel
#include <RendererInterface.h>               // IRendererModule
#include <RenderingThread.h>                 // ENQUEUE_RENDER_COMMAND, FlushRenderingCommands
#include <UObject/ObjectMacros.h>            // GENERATED_BODY, UCLASS, UFUNCTION

#include "SpCore/Unreal.h"

#include "SpUnrealTypes/SpRHIFeatureLevel.h" // ESpRHIFeatureLevel

#include "SpRendererModule.generated.h"

UCLASS()
class USpRendererModule : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void LoadPendingVirtualTextureTiles(ESpRHIFeatureLevel FeatureLevel)
    {
        ERHIFeatureLevel::Type feature_level = Unreal::getEnumValueAs<ERHIFeatureLevel::Type>(FeatureLevel);
        ENQUEUE_RENDER_COMMAND(SpLoadPendingVirtualTextureTiles)(
            [feature_level](FRHICommandListImmediate& rhi_command_list) {
                GetRendererModule().LoadPendingVirtualTextureTiles(rhi_command_list, feature_level);
            });
        FlushRenderingCommands();
    }
};
