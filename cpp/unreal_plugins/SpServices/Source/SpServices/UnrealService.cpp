//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include <memory> // std::make_unique, std::unique_ptr

#include "SpServices/EngineService.h"
#include "SpServices/EntryPointBinder.h"
#include "SpServices/RpcServer.h"

// UnrealService.h must only be included in cpp files, and must be included after all other headers.
#include "SpServices/UnrealService.h"

template <CUnrealEntryPointBinder TEntryPointBinder>
std::unique_ptr<UnrealService> UnrealService::create(TEntryPointBinder* unreal_entry_point_binder) { return std::make_unique<UnrealService>(unreal_entry_point_binder); }

template std::unique_ptr<UnrealService> UnrealService::create<EngineService<RpcServer>>(EngineService<RpcServer>*);
