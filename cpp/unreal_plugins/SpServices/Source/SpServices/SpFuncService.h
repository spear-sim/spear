//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <map>
#include <string>

#include <Components/SceneComponent.h>
#include <GameFramework/Actor.h>
#include <UObject/Object.h> // UObject

#include "SpCore/Assert.h"
#include "SpCore/SpArray.h"
#include "SpCore/SpFuncComponent.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/MsgpackAdaptors.h"
#include "SpServices/Service.h"
#include "SpServices/SharedMemoryService.h"

class SpFuncService : public Service
{
public:
    SpFuncService() = delete;
    SpFuncService(CUnrealEntryPointBinder auto* unreal_entry_point_binder, SharedMemoryService* shared_memory_service) : Service("SpFuncService")
    {
        SP_ASSERT(unreal_entry_point_binder);
        SP_ASSERT(shared_memory_service);

        shared_memory_service_ = shared_memory_service;

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("sp_func_service", "call_function", [this](uint64_t& uobject, std::string& function_name, SpFuncDataBundle& args) -> SpFuncDataBundle {

            UObject* uobject_ptr = toPtr<UObject>(uobject);
            USpFuncComponent* sp_func_component = getSpFuncComponent(uobject_ptr);

            // resolve references to shared memory
            SpArrayUtils::resolve(args.packed_arrays_, shared_memory_service_->getSharedMemoryViews());

            // validate args, call SpFunc, validate return values
            SpArrayUtils::validate(args.packed_arrays_, SpArraySharedMemoryUsageFlags::Arg);
            SpFuncDataBundle return_values = sp_func_component->callFunc(function_name, args);
            SpArrayUtils::validate(return_values.packed_arrays_, SpArraySharedMemoryUsageFlags::ReturnValue);

            return return_values;
        });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("sp_func_service", "get_shared_memory_views", [this](uint64_t& uobject) -> std::map<std::string, SpArraySharedMemoryView> {

            UObject* uobject_ptr = toPtr<UObject>(uobject);
            USpFuncComponent* sp_func_component = getSpFuncComponent(uobject_ptr);
            
            return sp_func_component->getSharedMemoryViews();
        });
    }

private:
    static USpFuncComponent* getSpFuncComponent(const UObject* uobject)
    {
        SP_ASSERT(uobject);

        USpFuncComponent* sp_func_component = nullptr;
        if (uobject->IsA(AActor::StaticClass())) {
            AActor* actor = const_cast<AActor*>(static_cast<const AActor*>(uobject));
            bool include_all_descendants = false;
            sp_func_component = Unreal::getChildComponentByType<AActor, USpFuncComponent>(actor, include_all_descendants);
        } else if (uobject->IsA(USceneComponent::StaticClass())) {
            USceneComponent* component = const_cast<USceneComponent*>(static_cast<const USceneComponent*>(uobject));
            bool include_all_descendants = false;
            sp_func_component = Unreal::getChildComponentByType<USceneComponent, USpFuncComponent>(component, include_all_descendants);
        } else {
            SP_ASSERT(false);
        }
        SP_ASSERT(sp_func_component);

        return sp_func_component;
    }

    SharedMemoryService* shared_memory_service_ = nullptr;
};
