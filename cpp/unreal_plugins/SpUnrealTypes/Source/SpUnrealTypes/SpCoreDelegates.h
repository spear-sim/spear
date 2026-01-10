//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/UnrealString.h> // FString
#include <HAL/Platform.h>            // int32, uint64
#include <Misc/CoreDelegates.h>
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SpCoreDelegates.generated.h"

UCLASS()
class USpCoreDelegates : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 
    UFUNCTION()
    static uint64 MountPak(const FString& PakFile, int32 PakOrder) { return reinterpret_cast<uint64>(FCoreDelegates::MountPak.Execute(PakFile, PakOrder)); }
};
