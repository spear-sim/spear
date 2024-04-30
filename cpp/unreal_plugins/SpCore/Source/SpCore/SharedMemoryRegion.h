//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stddef.h> // uint64_t

#include <string>

#include "SpCore/Boost.h"

struct SharedMemoryView
{
    std::string id_; // platform-dependent name used to access the shared memory resource from other processes
    int num_bytes_ = -1;
    void* data_ = nullptr;
};

class SPCORE_API SharedMemoryRegion
{
public:
    SharedMemoryRegion() = delete;
    SharedMemoryRegion(int num_bytes);              // useful for convenience
    SharedMemoryRegion(uint64_t id, int num_bytes); // useful if the caller wants to manage the allocation of uint64_t IDs to shared memory regions
    ~SharedMemoryRegion();

    const SharedMemoryView& getView();

private:
    static uint64_t getUniqueId();
    static std::string getUniqueIdString(uint64_t id);

    boost::interprocess::mapped_region mapped_region_;
    SharedMemoryView view_;
};
