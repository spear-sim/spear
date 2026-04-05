//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <map>
#include <ranges>  // std::views::transform
#include <string>
#include <utility> // std::make_pair
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // UWorld::InitializationValues
#include <HAL/Platform.h>                // SPSERVICES_API
#include <Misc/CoreDelegates.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SpArray.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

class SPSERVICES_API Service // SPSERVICES_API is needed here because the SpServicesEditor module needs to link against this class 
{
public:
    Service()
    {
        post_engine_init_handle_ = FCoreDelegates::OnPostEngineInit.AddRaw(this, &Service::postEngineInitHandler);
        engine_pre_exit_handle_ = FCoreDelegates::OnEnginePreExit.AddRaw(this, &Service::enginePreExitHandler);
        begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &Service::beginFrameHandler);
        end_frame_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &Service::endFrameHandler);
    }

    virtual ~Service()
    {
        FCoreDelegates::OnPostEngineInit.Remove(post_engine_init_handle_);
        FCoreDelegates::OnEnginePreExit.Remove(engine_pre_exit_handle_);
        FCoreDelegates::OnBeginFrame.Remove(begin_frame_handle_);
        FCoreDelegates::OnEndFrame.Remove(end_frame_handle_);

        post_engine_init_handle_.Reset();
        engine_pre_exit_handle_.Reset();
        begin_frame_handle_.Reset();
        end_frame_handle_.Reset();
    }

protected:
    virtual void postEngineInit() {}
    virtual void enginePreExit() {}
    virtual void beginFrame() {}
    virtual void endFrame() {}

    template <typename TValue>
    static uint64_t toUInt64(const TValue* src)
    {
        return reinterpret_cast<uint64_t>(src);
    }

    template <typename TValue>
    static std::vector<uint64_t> toUInt64(std::vector<TValue>&& src)
    {
        return Std::reinterpretAsVectorOf<uint64_t>(std::move(src));
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
    static std::vector<TPtr*> toPtr(std::vector<uint64_t>&& src)
    {
        return Std::reinterpretAsVectorOf<TPtr*>(std::move(src));
    }

    template <typename TKey, typename TPtr>
    static std::map<TKey, TPtr*> toPtr(const std::map<TKey, uint64_t>& src)
    {
        return Std::toMap<TKey, TPtr*>(src | std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, toPtr<TPtr>(value)); }));
    }

    template <typename TValue>
    static SpPackedArray toPackedArray(
        std::vector<TValue>&& data,
        const std::vector<int64_t>& shape,
        SpPackedArray& out_packed_array,
        const std::map<std::string, SpArraySharedMemoryView>& shared_memory_views,
        SpArraySharedMemoryUsageFlags usage_flags)
    {
        SpArray<TValue> array;

        // If an internal SpPackedArray was passed into a service entry as an out argument, then there is no
        // way to store information in it that will be visible to the client, so return a new internal SpPackedArray.

        if (out_packed_array.data_source_ == SpArrayDataSource::Internal) {
            array.setDataSource(std::move(data), shape); // set the data source of array to data

        // If a shared SpPackedArray was passed into a service entry as an out argument, then update array to
        // point to the user's shared memory, update the shared memory, and then return a new shared SpPackedArray.

        } else if (out_packed_array.data_source_ == SpArrayDataSource::Shared) {

            // Resolve out_packed_array's references to shared memory and validate that out_packed_array is
            // consistent with usage_flags.
            SpArrayUtils::resolve(out_packed_array, shared_memory_views);
            out_packed_array.validate(usage_flags);

            // Set the data source of array to the shared memory that is backing out_packed_array, and update
            // array's data values.
            array.setDataSource(shared_memory_views.at(out_packed_array.shared_memory_name_), SpArrayShapeUtils::getShape(shape, data.size()));
            array.setDataValues(data); // no move necessary because we're doing a memcpy into shared memory

        } else {
            SP_ASSERT(false);
        }

        return array.moveToPackedArray();
    }

private:
    void postEngineInitHandler()
    {
        postEngineInit();
    }

    void enginePreExitHandler()
    {
        enginePreExit();
    }

    void beginFrameHandler()
    {
        beginFrame();
    }

    void endFrameHandler()
    {
        endFrame();
    }

    FDelegateHandle post_engine_init_handle_;
    FDelegateHandle engine_pre_exit_handle_;
    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;
};
