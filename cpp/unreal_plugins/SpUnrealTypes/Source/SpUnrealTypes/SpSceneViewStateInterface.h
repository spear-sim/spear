//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <SceneManagement.h> // FSceneViewStateInterface

#include "SpSceneViewStateInterface.generated.h"

UCLASS()
class USpSceneViewStateInterface : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 
    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static uint32 GetPathTracingSampleIndex(uint64 ViewState)
    {
        #if RHI_RAYTRACING
            FSceneViewStateInterface* view_state_ptr = reinterpret_cast<FSceneViewStateInterface*>(ViewState);
            if (view_state_ptr) {
                return view_state_ptr->GetPathTracingSampleIndex();
            }
        #endif

        return 0;
    }

    UFUNCTION(Category="SPEAR") // uint64 is not supported for BlueprintCallable
    static uint32 GetPathTracingSampleCount(uint64 ViewState)
    {
        #if RHI_RAYTRACING
            FSceneViewStateInterface* view_state_ptr = reinterpret_cast<FSceneViewStateInterface*>(ViewState);
            if (view_state_ptr) {
                return view_state_ptr->GetPathTracingSampleCount();
            }
        #endif

        return 0;
    }
};