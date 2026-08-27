//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/InputService.h"

#include <memory> // std::make_unique, std::unique_ptr

#include "SpServices/EngineService.h"
#include "SpServices/EntryPointBinder.h"
#include "SpServices/RpcServer.h"

template <CUnrealEntryPointBinder TEntryPointBinder>
std::unique_ptr<InputService> InputService::create(TEntryPointBinder* unreal_entry_point_binder) { return std::make_unique<InputService>(unreal_entry_point_binder); }

template std::unique_ptr<InputService> InputService::create<EngineService<RpcServer>>(EngineService<RpcServer>*);
