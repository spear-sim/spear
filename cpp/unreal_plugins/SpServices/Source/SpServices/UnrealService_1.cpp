//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include <memory> // std::make_unique, std::unique_ptr

#include "SpCore/Assert.h"

#include "SpServices/EngineService.h"
#include "SpServices/EntryPointBinder.h"
#include "SpServices/RpcServer.h"

// UnrealService_1.h must only be included in cpp files, and must be included after all other headers.
#include "SpServices/UnrealService_1.h"

template <CUnrealEntryPointBinder TEntryPointBinder>
std::unique_ptr<UnrealService_1> UnrealService_1::create(TEntryPointBinder* unreal_entry_point_binder)
{
    std::unique_ptr<UnrealService_1> service = std::make_unique<UnrealService_1>(unreal_entry_point_binder);
    SP_ASSERT(service);
    return service;
}

template std::unique_ptr<UnrealService_1> UnrealService_1::create<EngineService<RpcServer>>(EngineService<RpcServer>*);
