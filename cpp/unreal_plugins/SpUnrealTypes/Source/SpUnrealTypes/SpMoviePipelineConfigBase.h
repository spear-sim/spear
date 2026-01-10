//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <MoviePipelineConfigBase.h>

#include "SpCore/Assert.h"

#include "SpMoviePipelineConfigBase.generated.h"

UCLASS()
class USpMoviePipelineConfigBase : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetDisplayName(UMoviePipelineConfigBase* Config, FString DisplayName)
    {
        SP_ASSERT(Config);
        Config->DisplayName = DisplayName;
    }
};
