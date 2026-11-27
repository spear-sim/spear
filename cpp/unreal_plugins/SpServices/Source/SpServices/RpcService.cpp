//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/RpcService.h"

#include <memory> // std::make_unique

#include "SpCore/Assert.h"

std::unique_ptr<RpcService> RpcService::create()
{
    std::unique_ptr<RpcService> service = std::make_unique<RpcService>();
    SP_ASSERT(service);
    return service;
}
