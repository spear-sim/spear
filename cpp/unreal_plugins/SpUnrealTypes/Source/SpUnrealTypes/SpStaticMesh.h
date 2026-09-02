//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/StaticMesh.h> // UStaticMesh
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SpCore/Assert.h"

#include "SpStaticMesh.generated.h"

class UBodySetup;

UCLASS()
class USpStaticMesh : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static UBodySetup* GetBodySetup(UStaticMesh* StaticMesh)
    {
        SP_ASSERT(StaticMesh);
        return StaticMesh->GetBodySetup();
    }
};
