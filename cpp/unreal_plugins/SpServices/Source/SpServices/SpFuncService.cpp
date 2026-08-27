//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/SpFuncService.h"

#include <memory> // std::make_unique, std::unique_ptr

#include "SpServices/EngineService.h"
#include "SpServices/EntryPointBinder.h"
#include "SpServices/RpcServer.h"
#include "SpServices/SharedMemoryService.h"

template <CUnrealEntryPointBinder TEntryPointBinder>
std::unique_ptr<SpFuncService> SpFuncService::create(TEntryPointBinder* unreal_entry_point_binder, SharedMemoryService* shared_memory_service) { return std::make_unique<SpFuncService>(unreal_entry_point_binder, shared_memory_service); }

template std::unique_ptr<SpFuncService> SpFuncService::create<EngineService<RpcServer>>(EngineService<RpcServer>*, SharedMemoryService*);
