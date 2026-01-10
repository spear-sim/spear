//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <AssetCompilingManager.h>
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SpAssetCompilingManager.generated.h"

UCLASS()
class USpAssetCompilingManager : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static int GetNumRemainingAssets() { return FAssetCompilingManager::Get().GetNumRemainingAssets(); }
};
