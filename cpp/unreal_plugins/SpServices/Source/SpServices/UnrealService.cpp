//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/UnrealService.h"

#include <string>
#include <vector>

#include <Engine/Engine.h> // GEngine
#include <Engine/World.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"

void UnrealService::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
        SP_ASSERT(!world_);
        world_ = world;
    }
}

void UnrealService::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    if (world == world_) {
        world_ = nullptr;
    }
}

UnrealServicePropertyDesc UnrealService::toServicePropertyDesc(const Unreal::PropertyDesc& property_desc)
{
    UnrealServicePropertyDesc service_property_desc;
    service_property_desc.property_ = reinterpret_cast<uint64_t>(property_desc.property_);
    service_property_desc.value_ptr_ = reinterpret_cast<uint64_t>(property_desc.value_ptr_);
    return service_property_desc;
}

Unreal::PropertyDesc UnrealService::toPropertyDesc(const UnrealServicePropertyDesc& service_property_desc)
{
    Unreal::PropertyDesc property_desc;
    property_desc.property_ = reinterpret_cast<FProperty*>(service_property_desc.property_);
    property_desc.value_ptr_ = reinterpret_cast<void*>(service_property_desc.value_ptr_);
    return property_desc;
}
