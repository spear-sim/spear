//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <PCGSubsystem.h>

#include "SpCore/Assert.h"

#include "SpPCGSubsystem.generated.h"

UCLASS()
class USpPCGSubsystem : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    bool IsInitialized(UPCGSubsystem* PCGSubsystem)
    {
        SP_ASSERT(PCGSubsystem);
        return PCGSubsystem->IsInitialized();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    void FlushCache(UPCGSubsystem* PCGSubsystem)
    {
        SP_ASSERT(PCGSubsystem);
        return PCGSubsystem->FlushCache();
    }
};
