//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Misc/App.h>
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SpApp.generated.h"

UCLASS()
class USpApp : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool IsBenchmarking()
    {
        return FApp::IsBenchmarking();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool SetBenchmarking(bool bVal)
    {
        FApp::SetBenchmarking(bVal);
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static double GetFixedDeltaTime()
    {
        return FApp::GetFixedDeltaTime();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetFixedDeltaTime(double Seconds)
    {
        FApp::SetFixedDeltaTime(Seconds);
    }
};
