//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/GameViewportClient.h>
#include <Kismet/BlueprintFunctionLibrary.h>
#include <Math/Vector2D.h>

#include "SpCore/Assert.h"

#include "SpGameViewportClient.generated.h"

UCLASS()
class USpGameViewportClient : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public: 
    UFUNCTION()
    static void GetViewportSize(UGameViewportClient* GameViewportClient, FVector2D& ViewportSize)
    {
        SP_ASSERT(GameViewportClient);
        return GameViewportClient->GetViewportSize(ViewportSize);
    }
};
