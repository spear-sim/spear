//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <map>
#include <string>
#include <vector>

#include <Components/SceneComponent.h>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>
#include <GameFramework/Actor.h>

#include "SpCore/Assert.h"
#include "SpCore/CppFuncComponent.h"
#include "SpCore/CppFuncData.h"
#include "SpCore/Log.h"
#include "SpCore/Rpclib.h"
#include "SpCore/Std.h"
#include "SpCore/SpCoreActor.h"
#include "SpCore/Unreal.h"
#include "SpCore/YamlCpp.h"
#include "SpEngine/EngineService.h"

// Needs to match SpCore/CppFuncData.h
enum class CppFuncServiceDataType
{
    Invalid    = -1,
    UInteger8  = 0,
    Integer8   = 1,
    UInteger16 = 2,
    Integer16  = 3,
    UInteger32 = 4,
    Integer32  = 5,
    UInteger64 = 6,
    Integer64  = 7,
    Float32    = 8,
    Float64    = 9
};
MSGPACK_ADD_ENUM(CppFuncServiceDataType);

struct CppFuncServiceData
{
    std::vector<uint8_t> data_;
    CppFuncServiceDataType data_type_ = CppFuncServiceDataType::Invalid;
    bool use_shared_memory_ = false;
    int shared_memory_num_bytes_ = -1;
    std::string shared_memory_name_;

    MSGPACK_DEFINE_MAP(data_, data_type_, use_shared_memory_, shared_memory_num_bytes_, shared_memory_name_);
};

struct CppFuncServiceArgs
{
    std::map<std::string, CppFuncServiceData> args_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string config_;

    MSGPACK_DEFINE_MAP(args_, unreal_obj_strings_, config_);
};

struct CppFuncServiceReturnValues
{
    std::map<std::string, CppFuncServiceData> return_values_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;

    MSGPACK_DEFINE_MAP(return_values_, unreal_obj_strings_, info_);
};

class CppFuncService {
public:
    CppFuncService() = delete;
    CppFuncService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &CppFuncService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &CppFuncService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncUnreal("cpp_func_service", "call_func", [this](const std::string& func_name, const CppFuncServiceArgs& args) -> CppFuncServiceReturnValues {
            SP_ASSERT(world_);

            UObject* object = Unreal::findActorByType<ASpCoreActor>(world_);
            SP_ASSERT(object);

            // get UCppFuncComponent
            UCppFuncComponent* cpp_func_component = nullptr;
            if (object->IsA(AActor::StaticClass())) {
                AActor* actor = const_cast<AActor*>(static_cast<const AActor*>(object));
                bool include_all_descendants = false;
                cpp_func_component = Unreal::getChildComponentByType<AActor, UCppFuncComponent>(actor, include_all_descendants);
            } else if (object->IsA(USceneComponent::StaticClass())) {
                USceneComponent* component = const_cast<USceneComponent*>(static_cast<const USceneComponent*>(object));
                bool include_all_descendants = false;
                cpp_func_component = Unreal::getChildComponentByType<USceneComponent, UCppFuncComponent>(component, include_all_descendants);
            } else {
                SP_ASSERT(false);
            }

            // prepare args
            CppFuncComponentArgs component_args;
            for (auto& [arg_name, arg] : args.args_) {
                CppFuncArg component_arg;
                component_arg.data_ = Std::toSpan(arg.data_);
                component_arg.data_type_ = static_cast<const CppFuncDataType>(arg.data_type_);
                Std::insert(component_args.args_, arg_name, std::move(component_arg));
            }
            component_args.unreal_obj_strings_ = args.unreal_obj_strings_;
            component_args.config_ = YAML::Load(args.config_);

            // call function
            CppFuncComponentReturnValues component_return_values = cpp_func_component->callFunc(func_name, component_args);

            // prepare return values
            CppFuncServiceReturnValues return_values;
            for (auto& [component_return_value_name, component_return_value] : component_return_values.return_values_) {
                CppFuncServiceData return_value;
                return_value.data_ = std::move(component_return_value.data_);
                return_value.data_type_ = static_cast<CppFuncServiceDataType>(component_return_value.data_type_);
                Std::insert(return_values.return_values_, component_return_value_name, std::move(return_value));
            }
            return_values.unreal_obj_strings_ = component_return_values.unreal_obj_strings_;
            YAML::Emitter emitter;
            emitter << component_return_values.info_;
            return_values.info_ = emitter.c_str();

            return return_values;
        });
    }

    ~CppFuncService()
    {
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);

private:
    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;
    UWorld* world_ = nullptr;
};
