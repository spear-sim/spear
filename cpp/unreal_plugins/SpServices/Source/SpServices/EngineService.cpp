//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/EngineService.h"

#include <memory> // std::make_unique, std::unique_ptr

#include "SpServices/EntryPointBinder.h"
#include "SpServices/RpcServer.h"

template <CEntryPointBinder TEntryPointBinder>
std::unique_ptr<EngineService<TEntryPointBinder>> EngineService<TEntryPointBinder>::create(TEntryPointBinder* entry_point_binder) { return std::make_unique<EngineService<TEntryPointBinder>>(entry_point_binder); }

template std::unique_ptr<EngineService<RpcServer>> EngineService<RpcServer>::create(RpcServer*);
