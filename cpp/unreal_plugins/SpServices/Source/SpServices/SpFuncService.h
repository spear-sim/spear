//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int8_t, uint8_t, uint64_t

#include <map>
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <UObject/Object.h>

#include "SpCore/Assert.h"
#include "SpCore/SharedMemoryRegion.h"
#include "SpCore/SpFuncArray.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpComponents/SpFuncComponent.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Msgpack.h"
#include "SpServices/Rpclib.h"
#include "SpServices/ServiceUtils.h"

#include "SpFuncService.generated.h"

class UWorld;

class SpFuncService {
public:
    SpFuncService() = delete;
    SpFuncService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SpFuncService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &SpFuncService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncNoUnreal("sp_func_service", "get_byte_order", []() -> std::string {
            SP_ASSERT(BOOST_ENDIAN_BIG_BYTE + BOOST_ENDIAN_LITTLE_BYTE == 1);
            
            if (BOOST_ENDIAN_BIG_BYTE) {
                return "big";
            } else if (BOOST_ENDIAN_LITTLE_BYTE) {
                return "little";
            } else {
                return "";
            }
        });

        unreal_entry_point_binder->bindFuncUnreal("sp_func_service", "call_function", [this](uint64_t& uobject, std::string& function_name, SpFuncDataBundle& args) -> SpFuncDataBundle {
            SP_ASSERT(world_);

            UObject* uobject_ptr = ServiceUtils::toPtr<UObject>(uobject);
            SP_ASSERT(uobject_ptr);

            // get SpFuncComponent and shared memory views
            USpFuncComponent* sp_func_component = getSpFuncComponent(uobject_ptr);
            const std::map<std::string, SpFuncSharedMemoryView>& shared_memory_views = sp_func_component->getSharedMemoryViews();

            // resolve references to shared memory and validate args
            SpFuncArrayUtils::resolve(args.packed_arrays_, shared_memory_views);

            // call SpFunc
            SpFuncArrayUtils::validate(args.packed_arrays_, SpFuncSharedMemoryUsageFlags::Arg);
            SpFuncDataBundle return_values = sp_func_component->callFunc(function_name, args);
            SpFuncArrayUtils::validate(return_values.packed_arrays_, SpFuncSharedMemoryUsageFlags::ReturnValue);
            
            return return_values;
        });

        unreal_entry_point_binder->bindFuncUnreal("sp_func_service", "get_shared_memory_views", [this](uint64_t& uobject) -> std::map<std::string, SpFuncSharedMemoryView> {
            SP_ASSERT(world_);

            UObject* uobject_ptr = ServiceUtils::toPtr<UObject>(uobject);
            SP_ASSERT(uobject_ptr);

            // get SpFuncComponent and return shared memory views
            USpFuncComponent* sp_func_component = getSpFuncComponent(uobject_ptr);
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

//
// SpFuncArrayDataSource
//

UENUM()
enum class ESpFuncArrayDataSource
{
    Invalid  = Unreal::getConstEnumValue(SpFuncArrayDataSource::Invalid),
    Internal = Unreal::getConstEnumValue(SpFuncArrayDataSource::Internal),
    External = Unreal::getConstEnumValue(SpFuncArrayDataSource::External),
    Shared   = Unreal::getConstEnumValue(SpFuncArrayDataSource::Shared)
};

//
// SpFuncArrayDataType
//

UENUM()
enum class ESpFuncArrayDataType
{
    Invalid = Unreal::getConstEnumValue(SpFuncArrayDataType::Invalid),
    u1      = Unreal::getConstEnumValue(SpFuncArrayDataType::UInt8),
    i1      = Unreal::getConstEnumValue(SpFuncArrayDataType::Int8),
    u2      = Unreal::getConstEnumValue(SpFuncArrayDataType::UInt16),
    i2      = Unreal::getConstEnumValue(SpFuncArrayDataType::Int16),
    u4      = Unreal::getConstEnumValue(SpFuncArrayDataType::UInt32),
    i4      = Unreal::getConstEnumValue(SpFuncArrayDataType::Int32),
    u8      = Unreal::getConstEnumValue(SpFuncArrayDataType::UInt64),
    i8      = Unreal::getConstEnumValue(SpFuncArrayDataType::Int64),
    f4      = Unreal::getConstEnumValue(SpFuncArrayDataType::Float32),
    f8      = Unreal::getConstEnumValue(SpFuncArrayDataType::Float64)
};

//
// SpFuncArrayDataType
//

UENUM(Flags)
enum class ESpFuncSharedMemoryUsageFlags
{
    DoNotUse    = Unreal::getConstEnumValue(SpFuncSharedMemoryUsageFlags::DoNotUse),
    Arg         = Unreal::getConstEnumValue(SpFuncSharedMemoryUsageFlags::Arg),
    ReturnValue = Unreal::getConstEnumValue(SpFuncSharedMemoryUsageFlags::ReturnValue)
};
ENUM_CLASS_FLAGS(ESpFuncSharedMemoryUsageFlags); // required if combining values using bitwise operations

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
        Msgpack::toObject(object, {
            {"packed_arrays", clmdep_msgpack::object(data_bundle.packed_arrays_, object.zone)},
            {"unreal_obj_strings", clmdep_msgpack::object(data_bundle.unreal_obj_strings_, object.zone)},
            {"info", clmdep_msgpack::object(data_bundle.info_, object.zone)}});
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
        packed_array.data_source_ = Unreal::getEnumValueFromStringAs<SpFuncArrayDataSource, ESpFuncArrayDataSource>(Msgpack::to<std::string>(map.at("data_source")));
        packed_array.shape_ = Msgpack::to<std::vector<uint64_t>>(map.at("shape"));
        packed_array.data_type_ = Unreal::getEnumValueFromStringAs<SpFuncArrayDataType, ESpFuncArrayDataType>(Msgpack::to<std::string>(map.at("data_type")));
        packed_array.shared_memory_name_ = Msgpack::to<std::string>(map.at("shared_memory_name"));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<SpFuncPackedArray> {
    void operator()(clmdep_msgpack::object::with_zone& object, SpFuncPackedArray const& packed_array) const {
        Msgpack::toObject(object, {
            {"data", clmdep_msgpack::object(packed_array.data_, object.zone)},
            {"data_source", clmdep_msgpack::object(Unreal::getStringFromEnumValue<ESpFuncArrayDataSource>(packed_array.data_source_), object.zone)},
            {"shape", clmdep_msgpack::object(packed_array.shape_, object.zone)},
            {"data_type", clmdep_msgpack::object(Unreal::getStringFromEnumValue<ESpFuncArrayDataType>(packed_array.data_type_), object.zone)},
            {"shared_memory_name", clmdep_msgpack::object(packed_array.shared_memory_name_, object.zone)}});
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
        shared_memory_view.usage_flags_ = Unreal::getCombinedEnumFlagValueFromStringsAs<SpFuncSharedMemoryUsageFlags, ESpFuncSharedMemoryUsageFlags>(Msgpack::to<std::vector<std::string>>(map.at("usage_flags")));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<SpFuncSharedMemoryView> {
    void operator()(clmdep_msgpack::object::with_zone& object, SpFuncSharedMemoryView const& shared_memory_view) const {
        Msgpack::toObject(object, {
            {"id", clmdep_msgpack::object(shared_memory_view.id_, object.zone)},
            {"num_bytes", clmdep_msgpack::object(shared_memory_view.num_bytes_, object.zone)},
            {"usage_flags", clmdep_msgpack::object(Unreal::getStringsFromCombinedEnumFlagValue<ESpFuncSharedMemoryUsageFlags, SpFuncSharedMemoryUsageFlags>(shared_memory_view.usage_flags_), object.zone)}});
    }
};
