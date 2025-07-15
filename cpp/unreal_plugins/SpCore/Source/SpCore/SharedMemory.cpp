//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SharedMemory.h"

#include <stddef.h> // uint64_t

#include <memory> // std::align

#include <string>

#include <boost/predef.h> // BOOST_OS_LINUX, BOOST_OS_MACOS, BOOST_OS_WINDOWS

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"

// TODO: remove platform-specific include
#if BOOST_COMP_MSVC
    #include <format>
#endif

void SharedMemory::initialize(uint64_t initial_unique_id)
{
    s_current_unique_id_ = initial_unique_id;
    s_initialized_ = true;
}

void SharedMemory::terminate()
{
    s_initialized_ = false;
}

uint64_t SharedMemory::getUniqueId()
{
    SP_ASSERT(s_initialized_);
    return s_current_unique_id_++;
}

std::string SharedMemory::getUniqueIdString(uint64_t id)
{
    // TODO: remove platform-specific logic
    #if BOOST_COMP_MSVC
        return std::format("__SP_SMEM_{:#018x}__", id);
    #elif BOOST_COMP_CLANG
        return (boost::format("__SP_SMEM_0x%016x__")%id).str(); // must be 30 chars or less
    #else
        #error
    #endif
}

SharedMemoryRegion::SharedMemoryRegion(uint64_t num_bytes) : SharedMemoryRegion(num_bytes, SharedMemory::getUniqueId()) {}

SharedMemoryRegion::SharedMemoryRegion(uint64_t num_bytes, uint64_t id)
{
    SP_ASSERT(num_bytes > 0);

    id_ = SharedMemory::getUniqueIdString(id);
    num_bytes_ = num_bytes;
    SP_ASSERT(id_ != "");

    uint64_t num_bytes_internal = num_bytes_ + s_alignment_padding_bytes_;

    #if BOOST_OS_WINDOWS
        boost::interprocess::windows_shared_memory windows_shared_memory(boost::interprocess::create_only, id_.c_str(), boost::interprocess::read_write, num_bytes_internal);
        mapped_region_ = boost::interprocess::mapped_region(windows_shared_memory, boost::interprocess::read_write);
    #elif BOOST_OS_MACOS || BOOST_OS_LINUX
        std::string native_id = "/" + id_; // add leading slash when creating, but not when referencing from Python
        boost::interprocess::shared_memory_object::remove(native_id.c_str());
        boost::interprocess::shared_memory_object shared_memory_object(boost::interprocess::create_only, native_id.c_str(), boost::interprocess::read_write);
        shared_memory_object.truncate(num_bytes_internal);
        mapped_region_ = boost::interprocess::mapped_region(shared_memory_object, boost::interprocess::read_write);
    #else
        #error
    #endif

    SP_ASSERT(mapped_region_.get_address());
}

SharedMemoryRegion::~SharedMemoryRegion()
{
    SP_ASSERT(id_ != "");
    #if BOOST_OS_MACOS || BOOST_OS_LINUX
        std::string native_id = "/" + id_; // add leading slash when removing
        boost::interprocess::shared_memory_object::remove(native_id.c_str());
    #endif
}

SharedMemoryView SharedMemoryRegion::getView()
{
    uint64_t num_bytes_internal = num_bytes_ + s_alignment_padding_bytes_;

    const void* const address = mapped_region_.get_address();
    SP_ASSERT(address);

    void* address_to_align = const_cast<void*>(address);
    size_t num_bytes_size_t = num_bytes_;
    size_t num_bytes_internal_size_t = num_bytes_internal;
    void* address_aligned = std::align(s_alignment_padding_bytes_, num_bytes_size_t, address_to_align, num_bytes_internal_size_t);
    SP_ASSERT(address_aligned);
    SP_ASSERT(address_aligned == address_to_align);

    boost::multiprecision::int128_t offset_bytes = static_cast<uint8_t*>(address_aligned) - static_cast<const uint8_t* const>(address);
    SP_ASSERT(offset_bytes >= 0);
    SP_ASSERT(offset_bytes < s_alignment_padding_bytes_);

    SharedMemoryView view;
    view.id_ = id_;
    view.num_bytes_ = num_bytes_;
    view.offset_bytes_ = static_cast<int16_t>(offset_bytes);
    view.data_ = address_aligned;
    return view;
}
