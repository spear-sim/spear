//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <HAL/Platform.h>   // uint64
#include <Kismet/BlueprintFunctionLibrary.h>
#include <UObject/Object.h> // UObject

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpFuncUtils.generated.h"

UCLASS()
class USpFuncUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static UObject* ToObject(FString String)
    {
        UObject* uobject = Std::toPtrFromString<UObject>(Unreal::toStdString(String));
        SP_ASSERT(uobject);
        return uobject;
    }
};
