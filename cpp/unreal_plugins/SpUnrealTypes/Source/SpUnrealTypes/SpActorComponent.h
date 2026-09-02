//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/ActorComponent.h>
#include <Kismet/BlueprintFunctionLibrary.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION

#include "SpCore/Assert.h"

#include "SpActorComponent.generated.h"

UCLASS()
class USpActorComponent : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void MarkRenderStateDirty(UActorComponent* ActorComponent)
    {
        SP_ASSERT(ActorComponent);
        ActorComponent->MarkRenderStateDirty();
    }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void SetCanEverAffectNavigation(UActorComponent* ActorComponent, bool bCanEverAffectNavigation)
    {
        SP_ASSERT(ActorComponent);
        ActorComponent->SetCanEverAffectNavigation(bCanEverAffectNavigation);
    }
};
