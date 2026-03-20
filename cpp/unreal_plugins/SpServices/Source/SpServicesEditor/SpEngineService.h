//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Kismet/BlueprintFunctionLibrary.h>
#include <Modules/ModuleManager.h>

#include "SpCore/Assert.h"

#include "SpServices/SpServices.h"

#include "SpEngineService.generated.h"

UCLASS()
class USpEngineService : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static void ExecuteEditorPostTransaction()
    {
        SpServices* sp_services = FModuleManager::Get().GetModulePtr<SpServices>("SpServices");
        SP_ASSERT(sp_services);
        SP_ASSERT(sp_services->engine_service_);
        sp_services->engine_service_->executeEditorPostTransaction();
    }
};
