//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/UnrealString.h> // FString
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SpDummyBlueprintFunctionLibrary.generated.h"

//
// The purpose of USpDummyBlueprintFunctionLibrary is to provide a minimal blueprint function library with
// all of the boilerplate required to implement a simple custom function that is accessible from Python. If
// you want to create a new blueprint function library for use with the SPEAR Python API, USpDummyBlueprintFunctionLibrary
// is a good starting point.
//

UCLASS()
class USpDummyBlueprintFunctionLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 

    // The BlueprintCallable decoration is not required for this function to be visible to the SPEAR Python
    // API, but it will make it so the function is also visible to Unreal's built-in Python API (which is
    // only available inside the editor) and Unreal's blueprint editor.
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static FString HelloWorld()
    {
        return "Hello World";
    }
};
