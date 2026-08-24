//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GroomAsset.h>
#include <Kismet/BlueprintFunctionLibrary.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION

#include "SpGroomLibrary.generated.h"

UCLASS()
class USpGroomLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    //
    // UGroomAsset::CacheDerivedDatas() synchronously builds a Groom's derived
    // "interpolation" render/simulation data. It is C++-only (not a UFUNCTION), so
    // it isn't reachable through SPEAR's generic UFUNCTION-call reflection path
    // without this wrapper -- and it matters that it IS reachable: using a
    // freshly-imported/reconfigured GroomAsset before this has run (attaching it to
    // a live GroomComponent, or even just letting the Editor's own deferred Content
    // Browser thumbnail generation glance at it) crashes with "Assertion failed:
    // !!(BulkData.Header.Flags & FHairStrandsInterpolationBulkData::DataFlags_HasData)".
    // Calling this directly and synchronously avoids that failure mode entirely,
    // instead of working around it with paced real-time ticks and Asset Editor
    // open/close tricks from Python.
    //
    // Editor-only (WITH_EDITORONLY_DATA), like the underlying engine function --
    // returns false in non-editor builds instead of failing to compile.
    //
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool CacheGroomDerivedDatas(UGroomAsset* Groom)
    {
#if WITH_EDITORONLY_DATA
        return Groom ? Groom->CacheDerivedDatas() : false;
#else
        return false;
#endif
    }
};
