//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GameFramework/GameModeBase.h>
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SpCore/Assert.h"

#include "SpGameModeBase.generated.h"

class APlayerController;

UCLASS()
class USpGameModeBase : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SwapPlayerControllers(AGameModeBase* GameModeBase, APlayerController* OldPC, APlayerController* NewPC)
    {
        SP_ASSERT(GameModeBase);
        GameModeBase->SwapPlayerControllers(OldPC, NewPC);
    }
};
