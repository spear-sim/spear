//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SharedMemoryRegion.h"

#include <stddef.h> // uint64_t

#include <string>

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"

// TODO: remove platform-specific include
#if BOOST_COMP_MSVC
    #include <format>
#endif

SharedMemoryRegion::SharedMemoryRegion(int num_bytes) : SharedMemoryRegion(getUniqueId(), num_bytes) {}

SharedMemoryRegion::SharedMemoryRegion(uint64_t id, int num_bytes)
{
    SP_ASSERT(num_bytes > 0);

    view_.id_ = getUniqueIdString(id);
    view_.num_bytes_ = num_bytes;

    #if BOOST_OS_WINDOWS
        boost::interprocess::windows_shared_memory windows_shared_memory(boost::interprocess::create_only, view_.id_.c_str(), boost::interprocess::read_write, num_bytes);
        mapped_region_ = boost::interprocess::mapped_region(windows_shared_memory, boost::interprocess::read_write);
    #elif BOOST_OS_MACOS || BOOST_OS_LINUX
        boost::interprocess::shared_memory_object::remove(view_.id_.c_str());
        boost::interprocess::shared_memory_object shared_memory_object(boost::interprocess::create_only, view_.id_.c_str(), boost::interprocess::read_write);
        shared_memory_object.truncate(num_bytes);
        mapped_region_ = boost::interprocess::mapped_region(shared_memory_object, boost::interprocess::read_write);
    #else
        #error
    #endif

    view_.data_ = mapped_region_.get_address();
}

SharedMemoryRegion::~SharedMemoryRegion()
{
    SP_ASSERT(view_.id_ != "");
    SP_ASSERT(view_.num_bytes_ > 0);
    SP_ASSERT(view_.data_);
    #if BOOST_OS_MACOS || BOOST_OS_LINUX
        boost::interprocess::shared_memory_object::remove(view_.id_.c_str());
    #endif
}

const SharedMemoryView& SharedMemoryRegion::getView()
{
    SP_ASSERT(view_.id_ != "");
    SP_ASSERT(view_.num_bytes_ > 0);
    SP_ASSERT(view_.data_);
    return view_;
}

uint64_t SharedMemoryRegion::getUniqueId()
{
    static uint64_t id = 0;
    return id++;    
}

std::string SharedMemoryRegion::getUniqueIdString(uint64_t id)
{
    // TODO: remove platform-specific logic
    #if BOOST_COMP_MSVC
        return std::format("sp_smem_{:20}", id); // don't use leading slash on Windows
    #elif BOOST_COMP_CLANG
        return (boost::format("/sp_smem_%20d")%id).str(); // use leading slash on macOS and Linux, must be 31 chars or less
    #else
        #error
    #endif
}
