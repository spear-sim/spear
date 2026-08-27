//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/UnrealService_0.h"

#include <memory> // std::make_unique, std::unique_ptr

#include "SpServices/EngineService.h"
#include "SpServices/EntryPointBinder.h"
#include "SpServices/RpcServer.h"

template <CUnrealEntryPointBinder TEntryPointBinder>
std::unique_ptr<UnrealService_0> UnrealService_0::create(TEntryPointBinder* unreal_entry_point_binder) { return std::make_unique<UnrealService_0>(unreal_entry_point_binder); }

template std::unique_ptr<UnrealService_0> UnrealService_0::create<EngineService<RpcServer>>(EngineService<RpcServer>*);
