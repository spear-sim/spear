//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int8_t, uint8_t, uint64_t

#include <map>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine

#include "SpCore/Assert.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/SpFuncArray.h"
#include "SpCore/Std.h"

#include "SpComponents/SpFuncComponent.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Msgpack.h"
#include "SpServices/Rpclib.h"

// TODO: remove these headers when ASpFuncServiceDebugActor is removed as the hard-coded target for function calls
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

// TODO: remove this header when ASpFuncServiceDebugActor is removed as the hard-coded target for function calls
#include "SpFuncService.generated.h"

class UObject;
class UWorld;

// TODO: remove this class as the hard-coded target for function calls
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ASpFuncServiceDebugActor : public AActor
{
    GENERATED_BODY()
public: 
    ASpFuncServiceDebugActor()
    {
        SP_LOG_CURRENT_FUNCTION();

        sp_func_component_ = Unreal::createComponentInsideOwnerConstructor<USpFuncComponent>(this, "sp_func_component");
        SP_ASSERT(sp_func_component_);

        sp_func_component_->registerFunc("hello_world", [this](SpFuncDataBundle& args) -> SpFuncDataBundle {
            SpFuncArray<uint8_t> hello("hello");
            SpFuncArray<double> my_data("my_data");

            hello.setData("Hello World!");
            my_data.setData({1.0, 2.0, 3.0});

            SpFuncDataBundle return_values;
            return_values.packed_arrays_ = SpFuncArrayUtils::moveToPackedArrays({hello.getPtr(), my_data.getPtr()});
            return return_values;
        });
    };

    ~ASpFuncServiceDebugActor()
    {
        SP_LOG_CURRENT_FUNCTION();
    };

private:
    USpFuncComponent* sp_func_component_ = nullptr;
};

class SpFuncService {
public:
    SpFuncService() = delete;
    SpFuncService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SpFuncService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &SpFuncService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncUnreal("sp_func_service", "call_func", [this](std::string& func_name, SpFuncDataBundle& args) -> SpFuncDataBundle {
            SP_ASSERT(world_);

            // TODO: make the object ptr an input to this function
            UObject* uobject = ASpFuncServiceDebugActor::StaticClass()->GetDefaultObject();
            SP_ASSERT(uobject);

            // get SpFuncComponent and shared memory views
            USpFuncComponent* sp_func_component = getSpFuncComponent(uobject);
            const std::map<std::string, SpFuncSharedMemoryView>& shared_memory_views = sp_func_component->getSharedMemoryViews();

            // resolve references to shared memory and validate args
            SpFuncArrayUtils::resolve(args.packed_arrays_, shared_memory_views);
            SpFuncArrayUtils::validate(args.packed_arrays_, SpFuncSharedMemoryUsageFlags::Arg);

            // call SpFunc
            SpFuncDataBundle return_values = sp_func_component->callFunc(func_name, args);

            // validate return values
            SpFuncArrayUtils::validate(return_values.packed_arrays_, SpFuncSharedMemoryUsageFlags::ReturnValue);
            
            return return_values;
        });

        unreal_entry_point_binder->bindFuncUnreal("sp_func_service", "get_shared_memory_views", [this]() -> std::map<std::string, SpFuncSharedMemoryView> {
            SP_ASSERT(world_);

            // TODO: make the object ptr an input to this function
            UObject* uobject = ASpFuncServiceDebugActor::StaticClass()->GetDefaultObject();
            SP_ASSERT(uobject);

            // get SpFuncComponent and return shared memory views
            USpFuncComponent* sp_func_component = getSpFuncComponent(uobject);
            return sp_func_component->getSharedMemoryViews();
        });
    }

    ~SpFuncService()
    {
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

private:
    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);

    static USpFuncComponent* getSpFuncComponent(const UObject* uobject);

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;
    UWorld* world_ = nullptr;
};

//
// Enums
//

MSGPACK_ADD_ENUM(SpFuncArrayDataSource);
MSGPACK_ADD_ENUM(SpFuncArrayDataType);
MSGPACK_ADD_ENUM(SpFuncSharedMemoryUsageFlags);

//
// SpFuncDataBundle
//

template <> // needed to receive a custom type as an arg
struct clmdep_msgpack::adaptor::convert<SpFuncDataBundle> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuncDataBundle& data_bundle) const {
        std::map<std::string, clmdep_msgpack::object> map = Msgpack::toMap(object);
        SP_ASSERT(map.size() == 3);
        data_bundle.packed_arrays_ = Msgpack::to<std::map<std::string, SpFuncPackedArray>>(map.at("packed_arrays"));
        data_bundle.unreal_obj_strings_ = Msgpack::to<std::map<std::string, std::string>>(map.at("unreal_obj_strings"));
        data_bundle.info_ = Msgpack::to<std::string>(map.at("info"));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<SpFuncDataBundle> {
    void operator()(clmdep_msgpack::object::with_zone& object, SpFuncDataBundle const& data_bundle) const {
        std::map<std::string, clmdep_msgpack::object> map = {
            {"packed_arrays", clmdep_msgpack::object(data_bundle.packed_arrays_, object.zone)},
            {"unreal_obj_strings", clmdep_msgpack::object(data_bundle.unreal_obj_strings_, object.zone)},
            {"info", clmdep_msgpack::object(data_bundle.info_, object.zone)}};
        Msgpack::toObject(object, map);
    }
};

//
// SpFuncPackedArray
//

template <> // needed to receive a custom type as an arg
struct clmdep_msgpack::adaptor::convert<SpFuncPackedArray> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuncPackedArray& packed_array) const {
        std::map<std::string, clmdep_msgpack::object> map = Msgpack::toMap(object);
        SP_ASSERT(map.size() == 5);
        packed_array.data_ = Msgpack::to<std::vector<uint8_t>>(map.at("data"));
        packed_array.data_source_ = Msgpack::to<SpFuncArrayDataSource>(map.at("data_source"));
        packed_array.shape_ = Msgpack::to<std::vector<uint64_t>>(map.at("shape"));
        packed_array.data_type_ = Msgpack::to<SpFuncArrayDataType>(map.at("data_type"));
        packed_array.shared_memory_name_ = Msgpack::to<std::string>(map.at("shared_memory_name"));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<SpFuncPackedArray> {
    void operator()(clmdep_msgpack::object::with_zone& object, SpFuncPackedArray const& packed_array) const {
        std::map<std::string, clmdep_msgpack::object> map = {
            {"data", clmdep_msgpack::object(packed_array.data_, object.zone)},
            {"data_source", clmdep_msgpack::object(packed_array.data_source_, object.zone)},
            {"shape", clmdep_msgpack::object(packed_array.shape_, object.zone)},
            {"data_type", clmdep_msgpack::object(packed_array.data_type_, object.zone)},
            {"shared_memory_name", clmdep_msgpack::object(packed_array.shared_memory_name_, object.zone)}};
        Msgpack::toObject(object, map);
    }
};

//
// SpFuncSharedMemoryView
//

template <> // needed to receive a custom type as an arg
struct clmdep_msgpack::adaptor::convert<SpFuncSharedMemoryView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SpFuncSharedMemoryView& shared_memory_view) const {
        std::map<std::string, clmdep_msgpack::object> map = Msgpack::toMap(object);
        SP_ASSERT(map.size() == 3);
        shared_memory_view.id_ = Msgpack::to<std::string>(map.at("id"));
        shared_memory_view.num_bytes_ = Msgpack::to<int>(map.at("num_bytes"));
        shared_memory_view.usage_flags_ = Msgpack::to<SpFuncSharedMemoryUsageFlags>(map.at("usage_flags"));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<SpFuncSharedMemoryView> {
    void operator()(clmdep_msgpack::object::with_zone& object, SpFuncSharedMemoryView const& shared_memory_view) const {
        std::map<std::string, clmdep_msgpack::object> map = {
            {"id", clmdep_msgpack::object(shared_memory_view.id_, object.zone)},
            {"num_bytes", clmdep_msgpack::object(shared_memory_view.num_bytes_, object.zone)},
            {"usage_flags", clmdep_msgpack::object(shared_memory_view.usage_flags_, object.zone)}};
        Msgpack::toObject(object, map);
    }
};
