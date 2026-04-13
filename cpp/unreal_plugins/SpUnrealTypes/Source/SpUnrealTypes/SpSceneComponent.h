//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/SceneComponent.h>
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SpSceneComponent.generated.h"

UCLASS()
class USpSceneComponent : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void MarkRenderTransformDirty(USceneComponent* SceneComponent)
    {
        SceneComponent->MarkRenderTransformDirty();
    }
};
