//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <SceneManagement.h>      // FSceneViewStateInterface
#include <RenderingThread.h>      // FlushRenderingCommands
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION

#include "SpCore/Assert.h"

#include "SpSceneViewStateInterface.generated.h"

UCLASS()
class USpSceneViewStateInterface : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 
    UFUNCTION()
    static uint32 GetPathTracingSampleIndex(uint64 ViewState) // uint64 not supported for BlueprintCallable
    {
        #if RHI_RAYTRACING
            FlushRenderingCommands(); // force rendering thread to be fully up-to-date before querying GetPathTracingSampleCount()
            FSceneViewStateInterface* view_state_ptr = reinterpret_cast<FSceneViewStateInterface*>(ViewState);
            SP_ASSERT(view_state_ptr);
            return view_state_ptr->GetPathTracingSampleIndex();
        #else
            SP_ASSERT(false);
            return 0;
        #endif
    }

    UFUNCTION(Category="SPEAR")
    static uint32 GetPathTracingSampleCount(uint64 ViewState) // uint64 not supported for BlueprintCallable
    {
        #if RHI_RAYTRACING
            FlushRenderingCommands(); // force rendering thread to be fully up-to-date before querying GetPathTracingSampleCount()
            FSceneViewStateInterface* view_state_ptr = reinterpret_cast<FSceneViewStateInterface*>(ViewState);
            SP_ASSERT(view_state_ptr);
            return view_state_ptr->GetPathTracingSampleCount();
        #else
            SP_ASSERT(false);
            return 0;
        #endif
    }
};
