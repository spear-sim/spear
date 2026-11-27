//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/EngineGlobalsService.h"

#include <memory> // std::make_unique, std::unique_ptr

#include "SpCore/Assert.h"

#include "SpServices/EngineService.h"
#include "SpServices/EntryPointBinder.h"
#include "SpServices/RpcServer.h"

template <CUnrealEntryPointBinder TEntryPointBinder>
std::unique_ptr<EngineGlobalsService> EngineGlobalsService::create(TEntryPointBinder* unreal_entry_point_binder)
{
    std::unique_ptr<EngineGlobalsService> service = std::make_unique<EngineGlobalsService>(unreal_entry_point_binder);
    SP_ASSERT(service);
    return service;
}

template std::unique_ptr<EngineGlobalsService> EngineGlobalsService::create<EngineService<RpcServer>>(EngineService<RpcServer>*);
