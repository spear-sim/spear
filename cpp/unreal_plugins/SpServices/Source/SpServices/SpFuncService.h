//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t, uint64_t

#include <map>
#include <memory> // std::make_unique, std::unique_ptr
#include <mutex>  // std::lock_guard
#include <string>
#include <vector>

#include <Components/SceneComponent.h>
#include <GameFramework/Actor.h>
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
#include "SpServices/Service.h"

#include "SpFuncService.generated.h"

class UWorld;

class SpFuncService : public Service {
public:
    SpFuncService() = delete;
    SpFuncService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("sp_func_service", "get_byte_order", []() -> std::string {
            SP_ASSERT(BOOST_ENDIAN_BIG_BYTE + BOOST_ENDIAN_LITTLE_BYTE == 1);
            if (BOOST_ENDIAN_BIG_BYTE) {
                return "big";
            } else if (BOOST_ENDIAN_LITTLE_BYTE) {
                return "little";
            } else {
                return "";
            }
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("sp_func_service", "create_shared_memory_region", [this](int& num_bytes, std::string& shared_memory_name) -> SpFuncSharedMemoryView {

            std::lock_guard<std::mutex> lock(mutex_);

            std::string name = shared_memory_name; // need to copy so we can pass to functions that expect a const ref
            SP_ASSERT(!Std::containsKey(shared_memory_regions_, name));
            SP_ASSERT(!Std::containsKey(shared_memory_views_, name));

            std::unique_ptr<SharedMemoryRegion> shared_memory_region = std::make_unique<SharedMemoryRegion>(num_bytes);
            SP_ASSERT(shared_memory_region);            
            SharedMemoryView view = shared_memory_region->getView();
            Std::insert(shared_memory_regions_, shared_memory_name, std::move(shared_memory_region));

            // assume that any shared memory region created from Python is an arg
            SpFuncSharedMemoryView shared_memory_view = SpFuncSharedMemoryView(view, SpFuncSharedMemoryUsageFlags::Arg);
            Std::insert(shared_memory_views_, shared_memory_name, shared_memory_view);

            return shared_memory_view;
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("sp_func_service", "destroy_shared_memory_region", [this](std::string& shared_memory_name) -> void {

            std::lock_guard<std::mutex> lock(mutex_);

            std::string name = shared_memory_name; // need to copy so we can pass to functions that expect a const ref
            SP_ASSERT(Std::containsKey(shared_memory_regions_, name));
            SP_ASSERT(Std::containsKey(shared_memory_views_, name));
            Std::remove(shared_memory_regions_, name);
            Std::remove(shared_memory_views_, name);
        });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("sp_func_service", "call_function", [this](uint64_t& uobject, std::string& function_name, SpFuncDataBundle& args) -> SpFuncDataBundle {

            std::lock_guard<std::mutex> lock(mutex_);

            UObject* uobject_ptr = toPtr<UObject>(uobject);
            USpFuncComponent* sp_func_component = getSpFuncComponent(uobject_ptr);

            // resolve references to shared memory and validate args, assume that shared memory views for args are in shared_memory_views_
            SpFuncArrayUtils::resolve(args.packed_arrays_, shared_memory_views_);

            // call SpFunc
            SpFuncArrayUtils::validate(args.packed_arrays_, SpFuncSharedMemoryUsageFlags::Arg);
            SpFuncDataBundle return_values = sp_func_component->callFunc(function_name, args);
            SpFuncArrayUtils::validate(return_values.packed_arrays_, SpFuncSharedMemoryUsageFlags::ReturnValue);
            
            return return_values;
        });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("sp_func_service", "get_shared_memory_views", [this](uint64_t& uobject) -> std::map<std::string, SpFuncSharedMemoryView> {

            // no synchronization needed here because this function doesn't access any member variables that
            // might be accessed concurrently

            UObject* uobject_ptr = toPtr<UObject>(uobject);
            USpFuncComponent* sp_func_component = getSpFuncComponent(uobject_ptr);
            
            return sp_func_component->getSharedMemoryViews();
        });
    }

    ~SpFuncService() = default;

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

    std::mutex mutex_; // used to coordinate read and write access to shared_memory_regions_ and shared_memory_views_
    std::map<std::string, std::unique_ptr<SharedMemoryRegion>> shared_memory_regions_;
    std::map<std::string, SpFuncSharedMemoryView> shared_memory_views_;
};

//
// Enums
//

//
// SpFuncArrayDataSource
//

UENUM()
enum class ESpFuncServiceArrayDataSource
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
enum class ESpFuncServiceArrayDataType
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
enum class ESpFuncServiceSharedMemoryUsageFlags
{
    DoNotUse    = Unreal::getConstEnumValue(SpFuncSharedMemoryUsageFlags::DoNotUse),
    Arg         = Unreal::getConstEnumValue(SpFuncSharedMemoryUsageFlags::Arg),
    ReturnValue = Unreal::getConstEnumValue(SpFuncSharedMemoryUsageFlags::ReturnValue)
};
ENUM_CLASS_FLAGS(ESpFuncServiceSharedMemoryUsageFlags); // required if combining values using bitwise operations

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
        packed_array.data_source_ = Unreal::getEnumValueFromStringAs<SpFuncArrayDataSource, ESpFuncServiceArrayDataSource>(Msgpack::to<std::string>(map.at("data_source")));
        packed_array.shape_ = Msgpack::to<std::vector<uint64_t>>(map.at("shape"));
        packed_array.data_type_ = Unreal::getEnumValueFromStringAs<SpFuncArrayDataType, ESpFuncServiceArrayDataType>(Msgpack::to<std::string>(map.at("data_type")));
        packed_array.shared_memory_name_ = Msgpack::to<std::string>(map.at("shared_memory_name"));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<SpFuncPackedArray> {
    void operator()(clmdep_msgpack::object::with_zone& object, SpFuncPackedArray const& packed_array) const {
        Msgpack::toObject(object, {
            {"data", clmdep_msgpack::object(packed_array.data_, object.zone)},
            {"data_source", clmdep_msgpack::object(Unreal::getStringFromEnumValueAs<ESpFuncServiceArrayDataSource>(packed_array.data_source_), object.zone)},
            {"shape", clmdep_msgpack::object(packed_array.shape_, object.zone)},
            {"data_type", clmdep_msgpack::object(Unreal::getStringFromEnumValueAs<ESpFuncServiceArrayDataType>(packed_array.data_type_), object.zone)},
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
        shared_memory_view.usage_flags_ = Unreal::getCombinedEnumFlagValueFromStringsAs<SpFuncSharedMemoryUsageFlags, ESpFuncServiceSharedMemoryUsageFlags>(Msgpack::to<std::vector<std::string>>(map.at("usage_flags")));
        return object;
    }
};

template <> // needed to send a custom type as a return value
struct clmdep_msgpack::adaptor::object_with_zone<SpFuncSharedMemoryView> {
    void operator()(clmdep_msgpack::object::with_zone& object, SpFuncSharedMemoryView const& shared_memory_view) const {
        Msgpack::toObject(object, {
            {"id", clmdep_msgpack::object(shared_memory_view.id_, object.zone)},
            {"num_bytes", clmdep_msgpack::object(shared_memory_view.num_bytes_, object.zone)},
            {"usage_flags", clmdep_msgpack::object(Unreal::getStringsFromCombinedEnumFlagValueAs<ESpFuncServiceSharedMemoryUsageFlags>(shared_memory_view.usage_flags_), object.zone)}});
    }
};
