//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>

#include "SpCore/Boost.h"

struct SharedMemoryView
{
    std::string name_; // user-provided globally unique name
    std::string id_;   // platform-dependent name used to access the shared memory resource
    int num_bytes_ = -1;
    void* data_ = nullptr;
};

class SPCORE_API SharedMemoryRegion
{
public:
    SharedMemoryRegion() = delete;
    SharedMemoryRegion(const std::string& name, int num_bytes);
    ~SharedMemoryRegion();

    const SharedMemoryView& getView();

private:
    boost::interprocess::mapped_region mapped_region_;
    SharedMemoryView view_;
};
