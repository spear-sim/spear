//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <string>

#include <HAL/Platform.h> // SPCORE_API

#include "SpCore/Boost.h"

class SharedMemory
{
public:
    SharedMemory() = delete;
    ~SharedMemory() = delete;

    static void initialize(uint64_t initial_unique_id);
    static void terminate();

    static uint64_t getUniqueId();
    static std::string getUniqueIdString(uint64_t unique_id);

private:
    inline static bool s_initialized_ = false;
    inline static uint64_t s_current_unique_id_ = 0;
};

struct SharedMemoryView
{
    std::string id_; // platform-dependent name used to access the shared memory resource from other processes
    uint64_t num_bytes_ = 0;
    uint16_t offset_bytes_ = 0;
    void* data_ = nullptr;
};

class SPCORE_API SharedMemoryRegion
{
public:
    SharedMemoryRegion() = delete;
    SharedMemoryRegion(uint64_t num_bytes);
    SharedMemoryRegion(uint64_t num_bytes, uint64_t id); // useful if the caller wants to manage the allocation of uint64_t IDs to shared memory regions
    ~SharedMemoryRegion();

    SharedMemoryView getView();

private:
    inline static constexpr uint16_t s_alignment_bytes_ = 4096;

    std::string id_;
    uint64_t num_bytes_ = 0;
    boost::interprocess::mapped_region mapped_region_;
};
