//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <Kismet2/BlueprintEditorUtils.h>

#include "SpBlueprintEditorUtils.generated.h"

class UBlueprint;

UCLASS()
class USpBlueprintEditorUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void PurgeNullGraphs(UBlueprint* Blueprint) { FBlueprintEditorUtils::PurgeNullGraphs(Blueprint); }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void ConformCallsToParentFunctions(UBlueprint* Blueprint) { FBlueprintEditorUtils::ConformCallsToParentFunctions(Blueprint); }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void ConformImplementedEvents(UBlueprint* Blueprint) { FBlueprintEditorUtils::ConformImplementedEvents(Blueprint); }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void ConformImplementedInterfaces(UBlueprint* Blueprint) { FBlueprintEditorUtils::ConformImplementedInterfaces(Blueprint); }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void UpdateOutOfDateCompositeNodes(UBlueprint* Blueprint) { FBlueprintEditorUtils::UpdateOutOfDateCompositeNodes(Blueprint); }

    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void UpdateTransactionalFlags(UBlueprint* Blueprint) { FBlueprintEditorUtils::UpdateTransactionalFlags(Blueprint); }
};
