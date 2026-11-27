//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/InitializeEngineService.h"

#include <memory> // std::make_unique

#include "SpCore/Assert.h"

std::unique_ptr<InitializeEngineService> InitializeEngineService::create()
{
    std::unique_ptr<InitializeEngineService> service = std::make_unique<InitializeEngineService>();
    SP_ASSERT(service);
    return service;
}
