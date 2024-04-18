//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int8_t, uint8_t

#include <map>
#include <string>
#include <vector>

#include <Components/SceneComponent.h>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>
#include <GameFramework/Actor.h>

#include "SpCore/Assert.h"
#include "SpCore/CppFunc.h"
#include "SpCore/CppFuncComponent.h"
#include "SpCore/Rpclib.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/Std.h"
#include "SpEngine/EntryPointBinder.h"

// TODO: remove these headers when SpCoreActor is removed as the hard-coded target for function calls
#include "SpCore/SpCoreActor.h"
#include "SpCore/Unreal.h"

// We use MSGPACK macros here to define structs that can be passed into, and returned from, the service entry
// points defined below. There are already similar struct defined in SpCore, but we choose to define separate
// structs here for the following reasons. First, we avoid a dependency on RPCLib in SpCore. Second, the data
// needed at the SpCore level is sometimes slightly different from the data needed at the SpEngine level, e.g.,
// CppFuncItem needs to maintain a view_ pointer but CppFuncServiceItem doesn't.

MSGPACK_ADD_ENUM(CppFuncDataType);
MSGPACK_ADD_ENUM(CppFuncSharedMemoryUsageFlags);

struct CppFuncServiceItem
{
    std::vector<uint8_t> data_;
    int num_elements_ = -1;
    CppFuncDataType data_type_ = CppFuncDataType::Invalid;
    bool use_shared_memory_ = false;
    std::string shared_memory_name_;

    MSGPACK_DEFINE_MAP(data_, num_elements_, data_type_, use_shared_memory_, shared_memory_name_);
};

struct CppFuncServiceItems
{
    std::map<std::string, CppFuncServiceItem> items_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;

    MSGPACK_DEFINE_MAP(items_, unreal_obj_strings_, info_);
};

struct CppFuncServiceSharedMemoryView
{
    std::string name_;
    std::string id_;
    int num_bytes_ = -1;
    CppFuncSharedMemoryUsageFlags usage_flags_;

    MSGPACK_DEFINE_MAP(name_, id_, num_bytes_, usage_flags_);
};

struct CppFuncServiceSharedMemoryViews
{
    std::map<std::string, CppFuncServiceSharedMemoryView> views_;

    MSGPACK_DEFINE_MAP(views_);
};

class CppFuncService {
public:
    CppFuncService() = delete;
    CppFuncService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &CppFuncService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &CppFuncService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncUnreal("cpp_func_service", "call_func", [this](const std::string& func_name, const CppFuncServiceItems& args) -> CppFuncServiceItems {
            SP_ASSERT(world_);

            // TODO: make the object ptr an input to this function
            UObject* object = Unreal::findActorByType<ASpCoreActor>(world_);
            SP_ASSERT(object);

            UCppFuncComponent* cpp_func_component = getCppFuncComponent(object);

            // prepare args
            CppFuncComponentItems component_args;
            for (auto& [arg_name, arg] : args.items_) {
                SP_ASSERT(arg_name != "");
                SP_ASSERT(arg.data_type_ != CppFuncDataType::Invalid);

                CppFuncItem component_arg;

                if (arg.use_shared_memory_) {
                    SP_ASSERT(arg.data_.empty());
                    SP_ASSERT(arg.num_elements_ >= 0);
                    SP_ASSERT(arg.shared_memory_name_ != "");

                    const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views = cpp_func_component->getSharedMemoryViews();
                    SP_ASSERT(Std::containsKey(shared_memory_views, arg.shared_memory_name_));
                    const CppFuncSharedMemoryView& shared_memory_view = shared_memory_views.at(arg.shared_memory_name_);
                    SP_ASSERT(shared_memory_view.view_.data_);
                    SP_ASSERT(arg.num_elements_* CppFuncDataTypeUtils::getSizeOf(arg.data_type_) <= shared_memory_view.view_.num_bytes_);
                    SP_ASSERT(shared_memory_view.usage_flags_ & CppFuncSharedMemoryUsageFlags::Arg);

                    component_arg.data_ = {};
                    component_arg.view_ = shared_memory_view.view_.data_;
                    component_arg.num_elements_ = arg.num_elements_;
                    component_arg.use_shared_memory_ = true;
                    component_arg.shared_memory_name_ = arg.shared_memory_name_;

                } else {
                    SP_ASSERT(arg.data_.size() % CppFuncDataTypeUtils::getSizeOf(arg.data_type_) == 0);
                    SP_ASSERT(arg.num_elements_ == -1);
                    SP_ASSERT(arg.shared_memory_name_ == "");

                    component_arg.data_ = std::move(arg.data_);
                    component_arg.view_ = component_arg.data_.data();
                    component_arg.num_elements_ = component_arg.data_.size() % CppFuncDataTypeUtils::getSizeOf(arg.data_type_);
                    component_arg.use_shared_memory_ = false;
                    component_arg.shared_memory_name_ = "";
                }

                component_arg.data_type_ = arg.data_type_;
                Std::insert(component_args.items_, arg_name, std::move(component_arg));
            }
            component_args.unreal_obj_strings_ = std::move(args.unreal_obj_strings_);
            component_args.info_ = std::move(args.info_);

            // call CppFunc
            CppFuncComponentItems component_return_values = cpp_func_component->callFunc(func_name, component_args);

            // prepare return values
            CppFuncServiceItems return_values;
            for (auto& [component_return_value_name, component_return_value] : component_return_values.items_) {
                SP_ASSERT(component_return_value_name != "");
                SP_ASSERT(component_return_value.data_type_ != CppFuncDataType::Invalid);

                CppFuncServiceItem return_value;

                if (component_return_value.use_shared_memory_) {
                    SP_ASSERT(component_return_value.data_.empty());
                    SP_ASSERT(component_return_value.num_elements_ >= 0);
                    SP_ASSERT(component_return_value.shared_memory_name_ != "");

                    const std::map<std::string, CppFuncSharedMemoryView>& shared_memory_views = cpp_func_component->getSharedMemoryViews();
                    SP_ASSERT(Std::containsKey(shared_memory_views, component_return_value.shared_memory_name_));
                    const CppFuncSharedMemoryView& shared_memory_view = shared_memory_views.at(component_return_value.shared_memory_name_);
                    SP_ASSERT(shared_memory_view.view_.data_);
                    SP_ASSERT(component_return_value.num_elements_*CppFuncDataTypeUtils::getSizeOf(component_return_value.data_type_) <= shared_memory_view.view_.num_bytes_);
                    SP_ASSERT(shared_memory_view.usage_flags_ & CppFuncSharedMemoryUsageFlags::ReturnValue);

                    return_value.data_ = {};
                    return_value.num_elements_ = component_return_value.num_elements_;
                    return_value.use_shared_memory_ = true;
                    return_value.shared_memory_name_ = component_return_value.shared_memory_name_;

                } else {
                    SP_ASSERT(component_return_value.data_.size() % CppFuncDataTypeUtils::getSizeOf(component_return_value.data_type_) == 0);
                    SP_ASSERT(component_return_value.num_elements_ == -1);
                    SP_ASSERT(component_return_value.shared_memory_name_ == "");

                    return_value.data_ = std::move(component_return_value.data_);
                    return_value.num_elements_ = return_value.data_.size() % CppFuncDataTypeUtils::getSizeOf(component_return_value.data_type_);
                    return_value.use_shared_memory_ = false;
                    return_value.shared_memory_name_ = "";
                }

                return_value.data_type_ = component_return_value.data_type_;
                Std::insert(return_values.items_, component_return_value_name, std::move(return_value));
            }
            return_values.unreal_obj_strings_ = std::move(component_return_values.unreal_obj_strings_);
            return_values.info_ = std::move(component_return_values.info_);

            return return_values;
        });

        unreal_entry_point_binder->bindFuncUnreal("cpp_func_service", "get_shared_memory_views", [this]() -> CppFuncServiceSharedMemoryViews {
            SP_ASSERT(world_);

            // TODO: make the object ptr an input to this function
            UObject* object = Unreal::findActorByType<ASpCoreActor>(world_);
            SP_ASSERT(object);

            UCppFuncComponent* cpp_func_component = getCppFuncComponent(object);

            CppFuncServiceSharedMemoryViews shared_memory_views;
            for (auto& [component_shared_memory_view_name, component_shared_memory_view] : cpp_func_component->getSharedMemoryViews()) {
                SP_ASSERT(component_shared_memory_view_name != "");

                CppFuncServiceSharedMemoryView shared_memory_view;
                shared_memory_view.name_ = component_shared_memory_view.view_.name_;
                shared_memory_view.id_ = component_shared_memory_view.view_.id_;
                shared_memory_view.num_bytes_ = component_shared_memory_view.view_.num_bytes_;
                shared_memory_view.usage_flags_ = component_shared_memory_view.usage_flags_;
                Std::insert(shared_memory_views.views_, component_shared_memory_view_name, shared_memory_view);
            }

            return shared_memory_views;
        });
    }

    ~CppFuncService()
    {
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

private:
    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);

    static UCppFuncComponent* getCppFuncComponent(UObject* object);

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;
    UWorld* world_ = nullptr;
};
