//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <AssetCompilingManager.h>
#include <UObject/Object.h> // UObject

#include "SpAssetCompilingManager.generated.h"

UCLASS()
class USpAssetCompilingManager : public UObject
{
    GENERATED_BODY()
public: 
    UFUNCTION()
    static int GetNumRemainingAssets() { return FAssetCompilingManager::Get().GetNumRemainingAssets(); }
};
