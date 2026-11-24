//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <memory>    // std::unique_ptr
#include <optional>
#include <stdexcept> // std::runtime_error
#include <string>

#include <HAL/Platform.h> // SPCORE_API

#include "SpCore/Boost.h"

struct SpSharedMemoryView // exposed to Python via SpServices/MsgpackAdaptors.h
{
    std::string id_; // platform-dependent name used to access the shared memory resource from other processes
    uint64_t num_bytes_ = 0;
    uint16_t offset_bytes_ = 0;
    void* data_ = nullptr;
};

// Acquires exclusive ownership of id for the lifetime of this object, regardless of how the application
// eventually shuts down. Throws SharedIdMutexError if id is already held by another process. The underlying
// OS-level lock is released automatically on process termination for any reason (including a crash), so id
// is always safe to reacquire on a subsequent run. The lock file backing this class is only removed if this
// object is destructed (i.e., if the application shuts down cleanly); otherwise it is reused (not recreated)
// and removed the next time an application successfully shuts down while holding id. Used internally by
// SharedMemoryRegion and SharedMemoryFence, to prevent independent processes from ever colliding on the
// same shared memory ID.

class SharedIdMutex
{
public:
    SharedIdMutex() = delete;
    SharedIdMutex(uint64_t id);
    ~SharedIdMutex();

private:
    static std::string getUniqueIdString(uint64_t id);

    std::string lock_file_path_;
    std::optional<boost::interprocess::file_lock> file_lock_;
};

class SharedIdMutexError : public std::runtime_error
{
public:
    SharedIdMutexError() = delete;
    SharedIdMutexError(uint64_t id, const std::string& id_string);

    uint64_t getId() const;
    std::string getIdString() const;

private:
    uint64_t id_ = 0;
    std::string id_string_;
};

class SPCORE_API SharedMemoryRegion
{
public:
    SharedMemoryRegion() = delete;
    SharedMemoryRegion(uint64_t num_bytes);
    SharedMemoryRegion(uint64_t num_bytes, uint64_t id); // useful if the caller wants to manage the allocation of uint64_t IDs to shared memory regions
    ~SharedMemoryRegion();

    SpSharedMemoryView getView();

private:
    inline static constexpr uint16_t s_alignment_bytes_ = 4096;

    SharedIdMutex shared_id_mutex_;
    std::string id_;
    uint64_t num_bytes_ = 0;
    boost::interprocess::mapped_region mapped_region_;
};

class SPCORE_API SharedMemoryFence
{
public:
    SharedMemoryFence();
    SharedMemoryFence(uint64_t id); // useful if the caller wants to manage the allocation of uint64_t IDs to shared memory regions

    SpSharedMemoryView getView();
    std::string getKey() const;
    void fence();

    // The fence region consists of a uint64_t counter, followed by a fixed-size, null-terminated buffer that
    // contains a key that uniquely identifies this fence (see SharedMemoryFence::SharedMemoryFence(...) in
    // SpCore/SharedMemory.cpp). Must match SharedMemoryFence in python_ext/cpp/shared_memory.h.
    inline static constexpr uint64_t s_key_num_bytes_ = 1024;

private:
    SharedMemoryRegion shared_memory_region_;
    uint64_t* counter_ = nullptr;
    std::string key_;
};

class SPCORE_API SharedMemory
{
public:
    SharedMemory() = delete;
    ~SharedMemory() = delete;

    static void initialize(uint64_t initial_unique_id);
    static void terminate();

    static SpSharedMemoryView getFenceView();
    static std::string getFenceKey();
    static void fence();

    static uint64_t getUniqueId();
    static std::string getUniqueIdString(uint64_t unique_id);

private:
    inline static bool s_initialized_ = false;
    inline static uint64_t s_current_unique_id_ = 0;
    inline static std::unique_ptr<SharedMemoryFence> s_shared_memory_fence_;
};
