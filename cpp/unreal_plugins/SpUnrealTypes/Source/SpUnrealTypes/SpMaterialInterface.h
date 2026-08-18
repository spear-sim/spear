//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/World.h>
#include <HAL/Platform.h>                 // uint8
#include <Kismet/BlueprintFunctionLibrary.h>
#include <MaterialShaderPrecompileMode.h> // EMaterialShaderPrecompileMode
#include <Materials/MaterialInterface.h>  // UMaterialInterface
#include <UObject/ObjectMacros.h>         // GENERATED_BODY, UCLASS, UENUM, UFUNCTION

#include "SpCore/Assert.h"
#include "SpCore/Unreal.h"

#include "SpMaterialInterface.generated.h"

//
// This enum corresponds to EMaterialShaderPrecompileMode declared in Engine/Source/Runtime/Engine/Public/MaterialShaderPrecompileMode.h
//

UENUM(BlueprintType)
enum class ESpMaterialShaderPrecompileMode : uint8
{
    None        = Unreal::getConstEnumValue(EMaterialShaderPrecompileMode::None),
    Background  = Unreal::getConstEnumValue(EMaterialShaderPrecompileMode::Background),
    Synchronous = Unreal::getConstEnumValue(EMaterialShaderPrecompileMode::Synchronous),
    Default     = Unreal::getConstEnumValue(EMaterialShaderPrecompileMode::Default)
};

UCLASS()
class USpMaterialInterface : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SubmitRemainingJobsForWorld(UWorld* World, ESpMaterialShaderPrecompileMode CompileMode)
    {
        SP_ASSERT(World);
        UMaterialInterface::SubmitRemainingJobsForWorld(World, Unreal::getEnumValueAs<EMaterialShaderPrecompileMode>(CompileMode));
    }
};
