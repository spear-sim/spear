//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <UObject/Package.h>

#include "SpCore/Assert.h"

#include "SpPackage.generated.h"

UCLASS()
class USpPackage : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsDirty(UPackage* Package)
    {
        SP_ASSERT(Package);
        return Package->IsDirty();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetDirtyFlag(UPackage* Package, bool bIsDirty)
    {
        SP_ASSERT(Package);
        Package->SetDirtyFlag(bIsDirty);
    }
};
