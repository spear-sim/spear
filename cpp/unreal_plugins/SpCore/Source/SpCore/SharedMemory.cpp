//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SharedMemory.h"

#include <stddef.h> // ptrdiff_t, size_t
#include <stdint.h> // uint64_t

#include <atomic>    // std::atomic_ref, std::memory_order_acq_rel
#include <filesystem>
#include <fstream>
#include <limits>    // std::numeric_limits
#include <memory>    // std::align, std::make_unique
#include <stdexcept> // std::runtime_error
#include <string>

#include <boost/predef.h> // BOOST_COMP_CLANG, BOOST_COMP_MSVC, BOOST_OS_LINUX, BOOST_OS_MACOS, BOOST_OS_WINDOWS

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"

// TODO: remove platform-specific include
#if BOOST_COMP_MSVC
    #include <format>
#endif

SharedIdMutex::SharedIdMutex(uint64_t id)
{
    std::string id_string = getUniqueIdString(id);
    std::filesystem::path lock_file_path = std::filesystem::temp_directory_path() / id_string;
    lock_file_path_ = lock_file_path.string();

    // file_lock requires the file to already exist; a leftover file from a previous unclean shutdown is
    // safe to reuse here, since the OS-level lock (not the file's existence) is what conveys ownership.
    if (!std::filesystem::exists(lock_file_path)) {
        std::ofstream(lock_file_path_).close();
    }

    file_lock_ = boost::interprocess::file_lock(lock_file_path_.c_str());

    if (!file_lock_->try_lock()) {
        throw SharedIdMutexError(id, id_string);
    }
}

SharedIdMutex::~SharedIdMutex()
{
    // Release the OS-level lock (and close the underlying file handle) before removing the file, rather
    // than relying on platform-specific behavior for removing a file that's still open.
    file_lock_.reset();
    std::filesystem::remove(lock_file_path_);
}

std::string SharedIdMutex::getUniqueIdString(uint64_t id)
{
    // TODO: remove platform-specific logic
    #if BOOST_COMP_MSVC
        return std::format("__SP_ID_MUTEX_{:#018x}__", id);
    #elif BOOST_COMP_CLANG
        return (boost::format("__SP_ID_MUTEX_0x%016x__")%id).str();
    #else
        #error
    #endif
}

SharedIdMutexError::SharedIdMutexError(uint64_t id, const std::string& id_string) : std::runtime_error(Std::toString("SharedIdMutexError: id=", id, " (id_string=", id_string, ") is already held by another process."))
{
    id_ = id;
    id_string_ = id_string;
}

uint64_t SharedIdMutexError::getId() const
{
    return id_;
}

std::string SharedIdMutexError::getIdString() const
{
    return id_string_;
}

SharedMemoryRegion::SharedMemoryRegion(uint64_t num_bytes) : SharedMemoryRegion(num_bytes, SharedMemory::getUniqueId()) {}

SharedMemoryRegion::SharedMemoryRegion(uint64_t num_bytes, uint64_t id) : shared_id_mutex_(id)
{
    SP_ASSERT(num_bytes > 0);
    SP_ASSERT(num_bytes <= std::numeric_limits<uint64_t>::max() - (s_alignment_bytes_ - 1));

    id_ = SharedMemory::getUniqueIdString(id);
    num_bytes_ = num_bytes;
    SP_ASSERT(id_ != "");

    uint64_t num_bytes_internal = num_bytes_ + s_alignment_bytes_ - 1;

    #if BOOST_OS_WINDOWS
        boost::interprocess::windows_shared_memory windows_shared_memory(boost::interprocess::create_only, id_.c_str(), boost::interprocess::read_write, num_bytes_internal);
        mapped_region_ = boost::interprocess::mapped_region(windows_shared_memory, boost::interprocess::read_write);
    #elif BOOST_OS_MACOS || BOOST_OS_LINUX
        std::string native_id = "/" + id_; // add leading slash when creating, but not when referencing from Python

        // A leftover shared memory object from a previous unclean shutdown is safe to remove here, since
        // shared_id_mutex_ guarantees that no other live process owns id.
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
    uint64_t num_bytes_internal = num_bytes_ + s_alignment_bytes_ - 1;

    const void* const address = mapped_region_.get_address();
    SP_ASSERT(address);

    void* address_to_align = const_cast<void*>(address);
    size_t num_bytes_size_t = num_bytes_;
    size_t num_bytes_internal_size_t = num_bytes_internal;
    void* address_aligned = std::align(s_alignment_bytes_, num_bytes_size_t, address_to_align, num_bytes_internal_size_t);
    SP_ASSERT(address_aligned);
    SP_ASSERT(address_aligned == address_to_align);

    std::ptrdiff_t offset_bytes = static_cast<uint8_t*>(address_aligned) - static_cast<const uint8_t* const>(address);
    SP_ASSERT(offset_bytes >= 0);
    SP_ASSERT(offset_bytes < s_alignment_bytes_);

    SharedMemoryView view;
    view.id_ = id_;
    view.num_bytes_ = num_bytes_;
    view.offset_bytes_ = static_cast<int16_t>(offset_bytes);
    view.data_ = address_aligned;
    return view;
}

SharedMemoryFence::SharedMemoryFence() : SharedMemoryFence(SharedMemory::getUniqueId()) {}

SharedMemoryFence::SharedMemoryFence(uint64_t id) : shared_memory_region_(sizeof(uint64_t), id)
{
    SP_ASSERT(std::atomic_ref<uint64_t>::is_always_lock_free);

    SharedMemoryView view = shared_memory_region_.getView();
    SP_ASSERT(view.data_);
    SP_ASSERT(Std::isPtrSufficientlyAligned(view.data_, std::atomic_ref<uint64_t>::required_alignment));

    counter_ = static_cast<uint64_t*>(view.data_);
}

SharedMemoryView SharedMemoryFence::getView()
{
    return shared_memory_region_.getView();
}

void SharedMemoryFence::fence()
{
    // SharedMemoryFence::fence() guarantees that all memory operations performed by one process before it calls
    // fence() happen-before all memory operations performed by another process after it calls fence(), as long
    // as the two fence() calls are already ordered in physical time* (i.e., as measured by a wall clock, as
    // opposed to any ordering defined by the C++ memory model) by some other mechanism (e.g., a synchronous RPC
    // call). In an RPC setting, this means that a client process' earlier writes are visible to a server
    // process' later reads, and the client's earlier reads complete before the server's later writes. Each pair
    // of fence() calls (i.e., one in the client, one in the server) establishes the guarantee in one direction
    // of a synchronous RPC call. Therefore, if a synchronous RPC call touches shared memory, it needs four
    // fence() calls in total. The client must call fence() before initiating the RPC call, the server must call
    // fence() before touching shared memory, the server must also call fence() after touching shared memory, and
    // the client must also call fence() after returning from the RPC call.
    //
    // Why a fence is necessary. Even though our RPC calls are fully synchronous, the socket communication that
    // orders the two processes is not formally guaranteed to establish a happens-before relationship between
    // memory operations in the two processes. So without an explicit fence, it is not strictly guaranteed that
    // one process observes the other process' most recent writes to shared memory, or that one process' reads
    // from shared memory complete before the other process overwrites the memory being read.
    //
    // How it works. fence() performs an atomic read-modify-write (RMW) with std::memory_order_acq_rel on a
    // counter that lives in a small SharedMemoryRegion. The value of the counter is never used; it only exists
    // so there is something to RMW. We increment by 1 rather than 0, so every fence() call is unambiguously a
    // write, because some compilers lower idempotent RMWs (e.g., fetch_add(0)) to a fence followed by a plain
    // load.
    //
    // Why it is correct. Consider a sender process that calls fence() and a receiver process that later calls
    // fence(), where the two calls are ordered in physical time* by RPC (i.e., within the sender process, the
    // sender's fence() call happens-before the sender sends an RPC message; the receiver can only receive that
    // message after (with respect to physical time*) it has been sent; and within the receiver process, the
    // receiver receiving that message happens-before the receiver's fence() call).
    //
    //     1. The sender's RMW happens-after** the sender's earlier memory operations, and the RMW is a release
    //        operation (due to std::memory_order_acq_rel). So everything that happens-before the RMW also
    //        happens-before anything that happens-after** an acquire operation that synchronizes-with the RMW.
    //     2. The receiver's RMW happens-before the receiver's later memory operations, and the RMW is an acquire
    //        operation (due to std::memory_order_acq_rel). So everything that happens-after** the RMW also
    //        happens-after** anything that happens-before the release operation that synchronizes-with the RMW.
    //     3. All modifications to the counter occur in a single total modification order, and a RMW always reads
    //        the last value in the modification order before its own write. Because the two fence() calls are
    //        ordered in physical time*, the sender's RMW completes before the receiver's starts in physical
    //        time*. The C++ standard does not formally require the modification order to be consistent with
    //        physical time*, but the hardware does, because cache coherence serializes all writes to a single
    //        memory location in the order in which they are performed (in physical time*), and the sender's RMW
    //        takes its place in that order before any of the sender's later instructions (e.g., the instructions
    //        that send the RPC message) take effect (see below). Therefore the sender's RMW precedes the
    //        receiver's in the modification order. So the receiver reads from the write performed by the sender,
    //        and the write it reads from is part of the release sequence headed by the sender's RMW, because a
    //        release sequence includes all subsequent RMWs to the same atomic object.
    //     4. Therefore the sender's RMW synchronizes-with the receiver's, so everything that happens-before the
    //        sender's fence() call also happens-before everything that happens-after** the receiver's fence()
    //        call.
    //
    // Why the sender's RMW can't be deferred. The sender's RMW doesn't participate in sending the RPC message,
    // so one might worry that the compiler or hardware could defer it until after the message has been sent, in
    // which case the receiver's RMW could precede the sender's RMW in the modification order. But this is not
    // allowed, because the sender's RMW is an acquire operation (due to std::memory_order_acq_rel), and an
    // acquire operation prevents all later memory operations in the same thread (including the ones that send
    // the RPC message) from being reordered before it, regardless of whether or not they depend on it. The
    // compiler must respect this constraint when emitting instructions, and the hardware must respect it when
    // executing them (e.g., on x86, LOCK XADD drains the store buffer and acts as a full barrier, and on ARM,
    // LDADDAL prevents later memory operations from being performed before it). Moreover, if the sender's RMW
    // were somehow deferred until after the receiver's RMW, then the sender's RMW would read from the receiver's
    // write, which would create a cycle: the receiver's RMW precedes the sender's RMW (because the sender's RMW
    // reads from it), the sender's RMW precedes the sender's message (because the sender's RMW is an acquire
    // operation), and the sender's message precedes the receiver's RMW (because the receiver receives the
    // message before calling fence()). Hardware memory models (e.g., x86 and ARM) forbid such cycles.
    //
    // Why it synchronizes all memory, not just the counter. The counter carries no information; it only serves
    // as a synchronization variable, in the same way that a mutex or a flag does. Once the sender's RMW
    // synchronizes-with the receiver's RMW, the C++ memory model guarantees that everything that happens-before
    // the sender's RMW also happens-before everything that happens-after** the receiver's RMW, regardless of
    // which memory locations are involved. In hardware terms, a release operation ensures that all earlier
    // memory operations in the same thread (to any memory location) are visible to all cores before the release
    // write is visible, and an acquire operation ensures that no later memory operations in the same thread (to
    // any memory location) are performed before the acquire read. So the physical time* argument is only needed
    // to establish that the receiver's RMW reads from the release sequence headed by the sender's RMW, and
    // everything else follows from ordinary release-acquire semantics.
    //
    // Why it is correct across processes. The C++ memory model formally describes threads within a single
    // program, but the standard also specifies that lock-free atomic operations should be address-free, i.e.,
    // they should work on memory that is shared between processes, even if it is mapped at different addresses
    // in each process (see [atomics.lockfree]). Mainstream compilers satisfy this requirement, and hardware
    // memory models (e.g., x86 and ARM) are defined in terms of physical memory across all cores, irrespective
    // of which process is running on each core. A lock-free fetch_add with std::memory_order_acq_rel compiles to
    // a single instruction (e.g., LOCK XADD on x86, LDADDAL on ARMv8.1+) that provides the required ordering. We
    // assert that std::atomic_ref<uint64_t> is always lock-free, so neither process can fall back to a
    // process-local lock. This is the same mechanism that interprocess mutexes and semaphores are built on.
    //
    // Cost. Each fence() call costs a single atomic instruction plus at most a cache line transfer between
    // cores, which is negligible compared to the cost of an RPC call. Unlike a mutex-based fence, nothing is
    // ever held, so an unclean shutdown of either process can't leave the fence in a state that blocks the other
    // process.
    //
    //      * Physical time is shorthand for the following more precise argument. There is no single global clock
    //        that all cores use. Instead, cache coherence ensures that only one core at a time can hold a given
    //        cache line for writing, so all writes to a given memory location occur in a single unambiguous
    //        order, and a RMW is performed entirely while its core holds the cache line. Moreover, sending and
    //        receiving an RPC message forms a chain of memory operations across cores (e.g., the sender's kernel
    //        writes to a socket buffer and wakes the receiver's thread using locks and memory barriers, and the
    //        receiver's kernel reads from the socket buffer), where each step is ordered by memory barriers.
    //        Hardware memory models (e.g., x86 and ARM) guarantee that such a chain is ordered across all cores,
    //        irrespective of which process is running on each core, and the order of writes to the counter must
    //        be consistent with this ordering. So in this discussion, when we say that A is ordered before B in
    //        physical time, what we mean precisely is that A is ordered before B by the hardware memory model,
    //        even though A and B are not ordered by the C++ memory model.
    //
    //     ** The C++ standard doesn't define the concept of happens-after, but we define it here to mean the
    //        converse of happens-before, i.e., if and only if event A happens-before event B, then event B
    //        happens-after event A.

    SP_ASSERT(counter_);
    std::atomic_ref<uint64_t>(*counter_).fetch_add(1, std::memory_order_acq_rel);
}

void SharedMemory::initialize(uint64_t initial_unique_id)
{
    s_current_unique_id_ = initial_unique_id;
    s_initialized_ = true;

    try {
        s_shared_memory_fence_ = std::make_unique<SharedMemoryFence>();
    } catch (const SharedIdMutexError& e) {
        SP_LOG("    ERROR: Couldn't acquire shared memory ID id=", e.getId(), " (id_string=", e.getIdString(), "). The Unreal Editor might be open already, or there might be another SPEAR executable running in the background. Close the Unreal Editor and other SPEAR executables, or change SP_CORE.SHARED_MEMORY_INITIAL_UNIQUE_ID to an unused ID, and try launching again.");
        SP_ASSERT(false);
    } catch (...) {
        SP_LOG("    ERROR: Unexpected error when attempting to acquire shared memory.");
        SP_ASSERT(false);
    }
}

void SharedMemory::terminate()
{
    s_shared_memory_fence_ = nullptr;
    s_initialized_ = false;
}

SharedMemoryView SharedMemory::getFenceView()
{
    SP_ASSERT(s_initialized_);
    SP_ASSERT(s_shared_memory_fence_);
    return s_shared_memory_fence_->getView();
}

void SharedMemory::fence()
{
    SP_ASSERT(s_initialized_);
    SP_ASSERT(s_shared_memory_fence_);
    s_shared_memory_fence_->fence();
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
