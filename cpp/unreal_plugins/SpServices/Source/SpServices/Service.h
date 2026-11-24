//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int64_t

#include <map>
#include <string>
#include <utility> // std::move
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <HAL/Platform.h>                // SPSERVICES_API
#include <Misc/CoreDelegates.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SpArray.h"

class SPSERVICES_API Service // SPSERVICES_API is needed here because the SpServicesEditor module needs to link against this class
{
public:
    Service()
    {
        begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &Service::beginFrameHandler);
        end_frame_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &Service::endFrameHandler);
    }

    virtual ~Service()
    {
        FCoreDelegates::OnBeginFrame.Remove(begin_frame_handle_);
        FCoreDelegates::OnEndFrame.Remove(end_frame_handle_);

        begin_frame_handle_.Reset();
        end_frame_handle_.Reset();
    }

protected:
    virtual void beginFrame() {}
    virtual void endFrame() {}

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
    void beginFrameHandler()
    {
        beginFrame();
    }

    void endFrameHandler()
    {
        endFrame();
    }

    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;
};
