//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t, uint64_t
#include <string.h> // strnlen

#include <atomic> // std::atomic_ref, std::memory_order_acq_rel
#include <string>

#include <boost/predef.h> // BOOST_OS_LINUX, BOOST_OS_MACOS, BOOST_OS_WINDOWS

#include "assert.h"
#include "boost.h"
#include "std.h"
#include "types.h"

// Client-side counterpart to SharedMemoryFence in SpCore/SharedMemory.h. The server owns the underlying shared
// memory region, and the client opens it by name using the SharedMemoryView returned by the server. See
// SharedMemoryFence::fence() in SpCore/SharedMemory.cpp for details on why the fence is necessary and why it
// is correct.

class SharedMemoryFence
{
public:
    SharedMemoryFence() = delete;

    // Throws boost::interprocess::interprocess_exception if the region can't be opened, e.g., if the server is
    // running on a different machine.
    SharedMemoryFence(const SharedMemoryView& view)
    {
        SP_ASSERT(std::atomic_ref<uint64_t>::is_always_lock_free);

        SP_ASSERT(view.id_ != "");
        SP_ASSERT(view.num_bytes_ == sizeof(uint64_t) + s_key_num_bytes_);

        #if BOOST_OS_WINDOWS
            boost::interprocess::windows_shared_memory windows_shared_memory(boost::interprocess::open_only, view.id_.c_str(), boost::interprocess::read_write);
            mapped_region_ = boost::interprocess::mapped_region(windows_shared_memory, boost::interprocess::read_write);
        #elif BOOST_OS_MACOS || BOOST_OS_LINUX
            std::string native_id = "/" + view.id_; // add leading slash when opening, see SharedMemoryRegion in SpCore/SharedMemory.cpp
            boost::interprocess::shared_memory_object shared_memory_object(boost::interprocess::open_only, native_id.c_str(), boost::interprocess::read_write);
            mapped_region_ = boost::interprocess::mapped_region(shared_memory_object, boost::interprocess::read_write);
        #else
            #error
        #endif

        SP_ASSERT(mapped_region_.get_address());
        SP_ASSERT(view.offset_bytes_ + view.num_bytes_ <= mapped_region_.get_size());

        uint8_t* data = static_cast<uint8_t*>(mapped_region_.get_address()) + view.offset_bytes_;
        SP_ASSERT(Std::isPtrSufficientlyAligned(data, std::atomic_ref<uint64_t>::required_alignment));

        counter_ = reinterpret_cast<uint64_t*>(data);
    }

    void fence()
    {
        SP_ASSERT(counter_);
        std::atomic_ref<uint64_t>(*counter_).fetch_add(1, std::memory_order_acq_rel);
    }

    std::string getKey()
    {
        SP_ASSERT(counter_);

        // We call fence() before reading the key, so the server's earlier write to the key is guaranteed to be
        // visible here, provided that the server called fence() after writing the key and before this call (e.g.,
        // by servicing any RPC call in between).
        fence();

        const char* key_buffer = reinterpret_cast<const char*>(counter_ + 1);
        return std::string(key_buffer, strnlen(key_buffer, s_key_num_bytes_));
    }

private:
    // Must match SharedMemoryFence in SpCore/SharedMemory.h.
    inline static constexpr uint64_t s_key_num_bytes_ = 1024;

    boost::interprocess::mapped_region mapped_region_;
    uint64_t* counter_ = nullptr;
};
