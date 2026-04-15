//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <HAL/Platform.h>   // int32
#include <Kismet/BlueprintFunctionLibrary.h>
#include <ShaderCompiler.h> // FShaderCompilingManager, GShaderCompilingManager

#include "SpCore/Assert.h"

#include "SpShaderCompilingManager.generated.h"

UCLASS()
class USpShaderCompilingManager : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsCompiling()
    {
        SP_ASSERT(GShaderCompilingManager);
        return GShaderCompilingManager->IsCompiling();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int32 GetNumRemainingJobs()
    {
        SP_ASSERT(GShaderCompilingManager);
        return GShaderCompilingManager->GetNumRemainingJobs();
    }
};
