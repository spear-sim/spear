//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SharedMemoryRegion.h"

#include <string>

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"

SharedMemoryRegion::SharedMemoryRegion(const std::string& name, int num_bytes)
{
    SP_ASSERT(name != "");
    SP_ASSERT(num_bytes > 0);

    view_.name_ = name;
    view_.num_bytes_ = num_bytes;

    #if BOOST_OS_WINDOWS
        view_.id_ = name; // don't use leading slash on Windows
        boost::interprocess::windows_shared_memory windows_shared_memory(
            boost::interprocess::create_only, view_.id_.c_str(), boost::interprocess::read_write, num_bytes);
        mapped_region_ = boost::interprocess::mapped_region(windows_shared_memory, boost::interprocess::read_write);
    #elif BOOST_OS_MACOS || BOOST_OS_LINUX
        view_.id_ = "/" + name; // use leading slash on macOS and Linux
        boost::interprocess::shared_memory_object::remove(view_.id_.c_str());
        boost::interprocess::shared_memory_object shared_memory_object(
            boost::interprocess::create_only, view_.id_.c_str(), boost::interprocess::read_write);
        shared_memory_object.truncate(num_bytes);
        mapped_region_ = boost::interprocess::mapped_region(shared_memory_object, boost::interprocess::read_write);
    #else
        #error
    #endif

    view_.data_ = mapped_region_.get_address();
}

SharedMemoryRegion::~SharedMemoryRegion()
{
    SP_ASSERT(view_.name_ != "");
    SP_ASSERT(view_.num_bytes_ > 0);
    SP_ASSERT(view_.data_);
#if BOOST_OS_MACOS || BOOST_OS_LINUX
        boost::interprocess::shared_memory_object::remove(view_.id_.c_str());
    #endif
}

const SharedMemoryView& SharedMemoryRegion::getView()
{
    SP_ASSERT(view_.name_ != "");
    SP_ASSERT(view_.num_bytes_ > 0);
    SP_ASSERT(view_.data_);
    return view_;
}
