//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/Array.h>                // TArray
#include <Containers/ArrayView.h>            // MakeArrayView
#include <Kismet/BlueprintFunctionLibrary.h>
#include <LandscapeSubsystem.h>              // ULandscapeSubsystem
#include <Math/Vector.h>                     // FVector
#include <UObject/ObjectMacros.h>            // GENERATED_BODY, UCLASS, UFUNCTION

#include "SpCore/Assert.h"

#include "SpLandscapeSubsystem.generated.h"

UCLASS()
class USpLandscapeSubsystem : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void RegenerateGrass(ULandscapeSubsystem* LandscapeSubsystem, bool bFlushGrass, bool bForceSync, bool bOverrideCameraLocations, TArray<FVector> CameraLocations)
    {
        SP_ASSERT(LandscapeSubsystem);
        if (bOverrideCameraLocations) {
            LandscapeSubsystem->RegenerateGrass(bFlushGrass, bForceSync, MakeArrayView(CameraLocations));
        } else {
            LandscapeSubsystem->RegenerateGrass(bFlushGrass, bForceSync);
        }
    }
};
