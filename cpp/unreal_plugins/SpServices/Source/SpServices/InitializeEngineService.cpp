//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/InitializeEngineService.h"

#include <memory> // std::make_unique

std::unique_ptr<InitializeEngineService> InitializeEngineService::create() { return std::make_unique<InitializeEngineService>(); }
