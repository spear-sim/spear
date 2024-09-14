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
    uint64_t num_bytes_ = -1;
    void* data_ = nullptr;
};

class SPCORE_API SharedMemoryRegion
{
public:
    SharedMemoryRegion() = delete;
    SharedMemoryRegion(int num_bytes);
    SharedMemoryRegion(int num_bytes, uint64_t id); // useful if the caller wants to manage the allocation of uint64_t IDs to shared memory regions
    ~SharedMemoryRegion();

    SharedMemoryView getView();

private:
    static uint64_t getUniqueId();
    static std::string getUniqueIdString(uint64_t id);

    std::string id_;
    uint64_t num_bytes_ = 0;
    boost::interprocess::mapped_region mapped_region_;
};
