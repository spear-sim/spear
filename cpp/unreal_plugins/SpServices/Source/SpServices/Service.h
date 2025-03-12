//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <map>
#include <ranges>  // std::views::transform
#include <string>
#include <utility> // std::make_pair
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/World.h>                // UWorld::InitializationValues

#include "SpCore/Std.h"
#include "SpCore/SpArray.h"

class Service {
public:
    Service();
    virtual ~Service();

protected:
    virtual void postWorldInitialization(UWorld* world, const UWorld::InitializationValues initialization_values);
    virtual void worldCleanup(UWorld* world, bool session_ended, bool cleanup_resources);
    virtual void worldBeginPlay();

    virtual void beginFrame() {}
    virtual void endFrame() {}

    UWorld* getWorld();

    template <typename TValue>
    static uint64_t toUInt64(const TValue* src)
    {
        return reinterpret_cast<uint64_t>(src);
    }

    template <typename TValue>
    static std::vector<uint64_t> toUInt64(const std::vector<TValue>& src)
    {
        return Std::reinterpretAsVectorOf<uint64_t>(src);
    }

    template <typename TKey, typename TValue>
    static std::map<TKey, uint64_t> toUInt64(const std::map<TKey, TValue>& src)
    {
        return Std::toMap<TKey, uint64_t>(src | std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, toUInt64(value)); }));
    }

    template <typename TPtr>
    static TPtr* toPtr(uint64_t src)
    {
        return reinterpret_cast<TPtr*>(src);
    }

    template <typename TPtr>
    static std::vector<TPtr*> toPtr(const std::vector<uint64_t>& src)
    {
        return Std::reinterpretAsVectorOf<TPtr*>(src);
    }

    template <typename TKey, typename TPtr>
    static std::map<TKey, TPtr*> toPtr(const std::map<TKey, uint64_t>& src)
    {
        return Std::toMap<TKey, TPtr*>(src | std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, toPtr<TPtr>(value)); }));
    }

    template <typename TValue>
    static SpPackedArray toPackedArray(
        const std::vector<TValue>& data,
        const std::vector<int64_t>& shape,
        SpPackedArray& packed_array,
        const std::map<std::string, SpArraySharedMemoryView>& shared_memory_views,
        SpArraySharedMemoryUsageFlags usage_flags)
    {
        SpArray<TValue> array;

        if (packed_array.data_source_ == SpArrayDataSource::Internal) {
            array.setDataSource(data, shape); // set the data source of array to data

        } else if (packed_array.data_source_ == SpArrayDataSource::Shared) {

            // resolve packed_array's references to shared memory and validate that packed_array is consistent with usage_flags
            SpArrayUtils::resolve(packed_array, shared_memory_views);
            packed_array.validate(usage_flags);

            // set the data source of array to the shared memory that is backing packed_array, and update array's data values to data
            array.setDataSource(shared_memory_views.at(packed_array.shared_memory_name_), SpArrayShapeUtils::getShape(shape, data.size()), packed_array.shared_memory_name_);
            array.setDataValues(data);

        } else {
            SP_ASSERT(false);
        }

        SpPackedArray return_value;
        array.moveToPackedArray(return_value);

        return return_value;
    }

private:
    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);
    void worldBeginPlayHandler();
    void beginFrameHandler();
    void endFrameHandler();

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;
    FDelegateHandle world_begin_play_handle_;
    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;

    UWorld* world_ = nullptr;
};
