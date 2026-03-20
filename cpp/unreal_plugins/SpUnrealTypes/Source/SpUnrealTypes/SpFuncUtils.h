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
    static int64 ToHandleFromObject(UObject* Object)
    {
        SP_ASSERT(Object);
        return reinterpret_cast<int64>(Object);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static UObject* ToObjectFromHandle(int64 Handle)
    {
        UObject* uobject = reinterpret_cast<UObject*>(Handle);
        SP_ASSERT(uobject);
        return uobject;
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString ToStringFromObject(UObject* Object)
    {
        SP_ASSERT(Object);
        return Unreal::toFString(Std::toStringFromPtr(Object));
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static UObject* ToObjectFromString(FString String)
    {
        UObject* uobject = Std::toPtrFromString<UObject>(Unreal::toStdString(String));
        SP_ASSERT(uobject);
        return uobject;
    }
};
