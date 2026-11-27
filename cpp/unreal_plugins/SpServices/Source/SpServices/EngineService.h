//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int64_t, uint64_t

#include <algorithm>   // std::swap
#include <array>
#include <atomic>
#include <chrono>      // std::chrono::duration, std::chrono::seconds, std::chrono::steady_clock
#include <condition_variable>
#include <exception>
#include <future>      // std::future_status, std::promise
#include <map>
#include <mutex>       // std::lock_guard, std::unique_lock
#include <ranges>      // std::views::transform
#include <string>
#include <thread>
#include <type_traits> // std::invoke_result_t, std::is_void_v
#include <utility>     // std::make_pair, std::move, std::pair
#include <vector>

#include <boost/predef.h> // BOOST_COMP_CLANG, BOOST_COMP_MSVC

#if BOOST_COMP_MSVC
    #include <format>
#endif

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/FuncInfo.h"
#include "SpServices/FuncSignatureRegistry.h"
#include "SpServices/Service.h"
#include "SpServices/SpTypes.h"
#include "SpServices/WorkQueue.h"

//
// EngineService implements the server side of the programming model described in the SPEAR paper (see the README for a
// link). The purpose of this overview is to connect that programming model with the code below.
//
// Entry points
//
// The primary responsibility of the SpServices plugin is to expose C++ functionality to Python. SpServices does so by
// binding named C++ lambda functions to an RPC server, and allowing the user's Python code to call these functions over
// an RPC connection. We refer to these lambda functions as entry points, and all communication with Python goes through
// these entry points. The RPC server executes entry points on a single worker thread inside the UE process, one
// function at a time, in the order the RPC calls arrive at the server. Additionally, the user's Python code calls our
// entry points one function at a time using blocking RPC calls. Therefore, the user's Python code and the RPC worker
// thread operate in lock-step, and the system behaves as if the user's Python code was synchronously calling into the
// worker thread within a single process. In contrast, the worker thread and the game thread run concurrently, and most
// of the code below is concerned with coordinating them.
//
// A pointer to the RPC server is passed directly into the EngineService constructor below. EngineService caches this
// pointer and exposes two high-level binding helper functions for the rest of the services in SpServices to use:
// bindFuncToExecuteOnWorkerThread(...) and bindFuncToExecuteOnGameThread(...). (In SPEAR, a service is simply a C++
// object that defines zero or more entry points, and whose lifetime matches the lifetime of the UE application.) Entry
// points that are passed into bindFuncToExecuteOnWorkerThread(...) execute directly on the worker thread. Entry points
// that are bound with bindFuncToExecuteOnGameThread(...) must execute on the game thread, so
// bindFuncToExecuteOnGameThread(...) wraps each such input function in a lambda function that is executed on the worker
// thread, and the wrapper lambda schedules the input function on a work queue that is eventually drained on the game
// thread. For each entry point that is passed into bindFuncToExecuteOnGameThread(...),
// bindFuncToExecuteOnGameThread(...) binds three distinct variants, each with a different synchronization policy, and a
// different prefix that is attached to the original name of the entry point. The synchronization policy of each variant
// is as follows:
//
//     - The call_sync  variant schedules the work, waits until the game thread has executed it, and returns its result.
//     - The call_async variant schedules the work and immediately returns a future object, whose result can be obtained
//                              later by calling the "engine_service.get_future_result_from_game_thread_as_<type>" entry
//                              point, which waits for the work to finish if it is still pending, and returns its
//                              result.
//     - The send_async variant schedules the work, immediately returns, and discards the result.
//
// Transactions and work queues
//
// In the SPEAR programming model, user Python code specifies a transaction of UE work by defining an
// instance.begin_frame() context followed by an instance.end_frame() context. Inside of these contexts, the user is
// allowed to call game thread entry points (i.e., bound with bindFuncToExecuteOnGameThread(...)), but is not allowed to
// call game thread entry points anywhere else. The user's Python code is allowed to call worker thread entry points
// (i.e., bound with bindFuncToExecuteOnWorkerThread(...)) anywhere. All work in the instance.begin_frame() context is
// guaranteed to execute on the game thread at the beginning of a UE frame, all work in the instance.end_frame() context
// is guaranteed to execute at the end of the same UE frame, and all work executes on the game thread in the order it
// was specified in the user's Python code. As the user's Python code enters and leaves these instance.begin_frame() and
// instance.end_frame() contexts, the SPEAR Python library internally calls the following worker thread entry points
// (see python/spear/services/engine_service.py):
//
//     - When entering an instance.begin_frame() context, the SPEAR Python library calls "engine_service.begin_frame".
//     - When exiting  an instance.begin_frame() context, the SPEAR Python library calls "engine_service.execute_frame".
//     - When exiting  an instance.end_frame()   context, the SPEAR Python library calls "engine_service.end_frame".
//
// In order to execute the incoming work items from the user's Python program on the UE game thread, EngineService
// maintains two distinct work queues to store the incoming work, as well as a current work queue pointer; one of these
// queues is for instance.begin_frame() work and the other is for instance.end_frame() work. As the user's Python
// program calls "engine_service.begin_frame", "engine_service.execute_frame", and "engine_service.end_frame",
// EngineService updates the current work queue pointer. As the user's Python program calls other entry points (e.g.,
// "unreal_service.spawn_actor"), the current work queue is populated with work items. To drain the queues,
// EngineService receives callbacks from the UE game thread at the beginning and the end of each UE frame, and
// EngineService drains the appropriate queue in each callback.
//
// The game thread executes each queue indefinitely, waiting for new work items to come in until a break signal is
// queued and executed. These break signals are queued by EngineService in "engine_service.execute_frame" and
// "engine_service.end_frame". If the user's Python code is too far ahead of the game thread, then the
// "engine_service.begin_frame" and "engine_service.execute_frame" entry points will block until the game thread has a
// chance to catch up. By default, the user's Python code is allowed to get one full frame ahead of the UE game thread,
// meaning that the user's Python code is allowed to have executed one full instance.begin_frame() context and one full
// instance.end_frame() context before the game thread has started executing the instance.begin_frame() work.
// Optionally, EngineService can maintain two sets of queues and alternate between them, which allows the user's Python
// code to get two full frames ahead of the game thread instead of one. We refer to this approach as double-buffered
// mode in the code below.
//
// If there is no pending transaction when the game thread reaches the beginning of a UE frame, then by default the game
// thread doesn't wait, and UE executes the frame normally (i.e., without involving any of EngineService's work queues).
// An important consequence of this design is that consecutive user transactions are not guaranteed to execute on
// consecutive UE frames. This is intentional, because it enables the UE application to remain responsive in between
// user transactions, e.g., the user can interact with the Unreal Editor while their Python program is paused in between
// transactions. If desired, it is possible to prevent the game thread from advancing in between transactions by calling
// instance.end_frame(single_step=True), which guarantees that consecutive transactions execute on consecutive frames.
//
// The cooperative execution model described here enables two distinct use cases. First, user Python code can
// conveniently specify complex graphs of synchronous UE work, and the user can reason precisely about when and how
// their graphs will execute on the game thread. Second, user Python code can conveniently specify streams of
// asynchronous UE work in a way that is maximally efficient and completely avoids slowing down the game thread.
//
// Editor transactions
//
// SPEAR programs can be executed with minimal modifications as editor Python scripts inside the Unreal Editor's
// embedded Python environment. In this case, the user's Python code runs on a Python interpreter that executes directly
// on the game thread, and SPEAR functions communicate with the RPC worker thread using the execution model described
// above. But if the Python interpreter executes directly on the game thread, then our execution model is guaranteed to
// deadlock when calling the call_sync variant of any game thread entry point. This is because the Python thread (i.e.,
// the game thread) will synchronously call the worker thread wrapper for the entry point and wait for it to return.
// Meanwhile, the worker thread wrapper will schedule the work on a queue that must be drained on the game thread and
// wait for the game thread to drain it. But the game thread can't drain the queue because it is waiting for the worker
// thread wrapper to return.
//
// To avoid this deadlock when operating inside the Unreal Editor's embedded Python environment, the SPEAR Python
// library executes each game thread function call in four steps, all of which happen internally so the user's code
// doesn't need to be aware of them. First, the SPEAR Python library enters into a special editor transaction context
// that sets the current queue to be a dedicated editor queue. Second, the Python SPEAR Python library calls the
// call_async variant of the entry point, which schedules the work on the dedicated editor queue and immediately returns
// a future. Third, the SPEAR Python library calls EngineService::executeEditorPostTransaction() directly on the game
// thread (i.e., not through our RPC server), which drains a single work item from the editor queue and stores the
// result for later retrieval. Fourth, the SPEAR python library retrieves the result on the worker thread and returns it
// to the user's code, which is guaranteed to return immediately because the work has already executed.
//
// Error handling
//
// If an exception occurs in a game thread or worker thread entry point during a transaction, EngineService enters an
// error state. However, EngineService only reports its error state to Python at well-defined points in the transaction,
// (e.g., via the return values of "engine_service.execute_frame" and "engine_service.end_frame"). So, if the user's
// Python code is in the middle of a transaction when the error occurs, the SPEAR Python library can't stop it
// immediately, and the user's Python code will continue calling entry points until it reaches the end of its current
// instance.begin_frame() or instance.end_frame() context. To handle these in-flight calls despite being in an error
// state, the wrapper lambdas that wrap each game-thread entry point immediately return default-constructed values
// without scheduling any additional work. Likewise, any work that was already scheduled before the error occurred
// immediately returns a default-constructed value when it executes on the game thread. Eventually, once the error state
// has propagated to Python, the SPEAR Python library will conclude the transaction and assert. By design, the only way
// to reset the server's error state is to call the "engine_service.initialize" entry point, which will happen
// automatically when the user's Python code creates a new instance.
//
// If the user's Python code exits in the middle of a transaction, the game thread can be stalled indefinitely, waiting
// for work that will never arrive. To handle this case, a watchdog thread enters the error state and releases the game
// thread if it has been waiting for too long. Additionally, "engine_service.initialize" and "engine_service.terminate"
// enter the error state and release the game thread immediately, so a new instance can always connect to a stalled
// server.
//

template <CEntryPointBinder TEntryPointBinder>
class EngineService : public Service
{
public:
    EngineService() = delete;
    EngineService(TEntryPointBinder* entry_point_binder)
    {
        SP_ASSERT(entry_point_binder);

        entry_point_binder_ = entry_point_binder;

        //
        // Entry points for initializing and terminating the frame state.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "initialize", [this]() -> void {

            // Make sure the game thread isn't using any batched work queues before we re-initialize them.
            releaseGameThread();

            // Prevent the watchdog thread from accessing the work queues or setting the error state, and prevent
            // shutdown() from terminating the work queues, while we re-initialize them. We acquire this lock after
            // calling releaseGameThread() rather than before, to avoid holding it while waiting on the game thread.
            std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

            // If shutdown() has already been called, then the work queues have been terminated, and we must not
            // re-initialize them, because the game thread will never execute them, so any RPC handler that
            // schedules work on them would wait forever.
            if (shutdown_) {
                SP_LOG("worker_thread: ERROR: engine_service.initialize was called after EngineService::shutdown(), returning without re-initializing...");
                return;
            }

            // Reset frame state.
            frame_state_ = FrameState::Idle;

            // Reset request flags.
            request_terminate_ = false;
            request_error_ = false;

            // Reset state for queue management.

            {
                std::lock_guard<std::mutex> lock(batched_work_queue_mutex_);
                batched_work_queue_fifo_.clear();
            }

            current_queuing_modes_.clear();
            current_batched_work_queues_.clear();
            current_incremental_work_queues_.clear();

            bool double_buffered_work_queues = Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.DOUBLE_BUFFERED_WORK_QUEUES");

            if (double_buffered_work_queues) {
                batched_work_queue_fifo_size_    = BATCHED_WORK_QUEUE_FIFO_SIZE_DOUBLE_BUFFERED;
                begin_frame_writable_by_wt_      = static_cast<int>(BatchedQueues::BeginFrameA);
                end_frame_writable_by_wt_        = static_cast<int>(BatchedQueues::EndFrameA);
                begin_frame_being_drained_by_gt_ = static_cast<int>(BatchedQueues::BeginFrameB);
                end_frame_being_drained_by_gt_   = static_cast<int>(BatchedQueues::EndFrameB);
            } else {
                batched_work_queue_fifo_size_    = BATCHED_WORK_QUEUE_FIFO_SIZE_SINGLE_BUFFERED;
                begin_frame_writable_by_wt_      = static_cast<int>(BatchedQueues::BeginFrameA);
                end_frame_writable_by_wt_        = static_cast<int>(BatchedQueues::EndFrameA);
                begin_frame_being_drained_by_gt_ = static_cast<int>(BatchedQueues::BeginFrameA);
                end_frame_being_drained_by_gt_   = static_cast<int>(BatchedQueues::EndFrameA);
            }

            // WorkQueue::initialize() replaces the queue's io_context, which is only safe if the game thread isn't
            // inside WorkQueue::run() or WorkQueue::pollOne(). The game thread isn't inside run() on any batched
            // work queue, because we called releaseGameThread() above, and it isn't inside pollOne() on the editor
            // transaction work queue, because we're not inside an editor transaction.

            for (int i = 0; i < NUM_BATCHED_QUEUES; i++) {
                batched_work_queues_.at(i).initialize();
            }

            editor_transaction_work_queue_.initialize();

            {
                std::lock_guard<std::mutex> lock(batched_work_queue_mutex_);
                for (int i = 0; i < NUM_BATCHED_QUEUES; i++) {
                    batched_work_queue_ready_promises_.at(i) = std::promise<void>();
                    batched_work_queue_ready_futures_.at(i) = batched_work_queue_ready_promises_.at(i).get_future();
                    batched_work_queue_ready_promises_.at(i).set_value();
                    batched_work_queue_ready_promises_set_.at(i) = true;
                }
            }
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "terminate", [this]() -> void {

            // The editor transaction work queue isn't covered by releaseGameThread(), but it is not in use, because
            // engine_service.terminate is not allowed to be called inside an editor transaction.
            SP_ASSERT(current_incremental_work_queues_.empty());

            // Make sure the game thread isn't using any batched work queues before we terminate them.
            releaseGameThread();

            // Prevent the watchdog thread from accessing the work queues while we terminate them. We acquire
            // this lock after calling releaseGameThread() rather than before, to avoid holding it while waiting
            // on the game thread.
            std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

            // Set the frame state to prevent the game thread from executing any more work queues.
            frame_state_ = FrameState::Terminating;

            // WorkQueue::terminate() destroys the queue's io_context, which is only safe if the game thread isn't
            // inside WorkQueue::run() or WorkQueue::pollOne(). The game thread isn't inside run() on any batched
            // work queue, because we called releaseGameThread() above, and it isn't inside pollOne() on the editor
            // transaction work queue, because we're not inside an editor transaction.

            for (int i = 0; i < NUM_BATCHED_QUEUES; i++) {
                batched_work_queues_.at(i).terminate();
            }

            editor_transaction_work_queue_.terminate();

            // Indicate to the game thread that we want to terminate the application.
            request_terminate_ = true;
        });

        //
        // Entry points for querying other entry points.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "get_entry_point_signature_type_names", [this]() -> std::vector<std::string> {
            return FuncSignatureRegistry::getTypeNames();
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "get_entry_point_signature_descs", [this]() -> std::map<std::string, SpFuncSignatureDesc> {
            return func_signature_registry_.getFuncSignatureDescs();
        });

        //
        // Entry points for managing the frame state.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "begin_frame", [this]() -> bool {

            SP_ASSERT(!request_terminate_);

            if (request_error_) {
                return false;
            }

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("worker_thread: engine_service.begin_frame: Waiting for beginFrame() to finish executing...");
            }

            // Wait until beginFrame() on frame i-1 (or i-2 if we're in double-buffered mode) has finished
            // executing before we allow the user to start queuing work on the queue for frame i.
            batched_work_queue_ready_futures_.at(begin_frame_being_drained_by_gt_).get();

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("worker_thread: engine_service.begin_frame: Finished waiting.");
            }

            // Reset promise and future and swap read/write indices.

            {
                std::lock_guard<std::mutex> lock(batched_work_queue_mutex_);

                // Now that the being-drained-by-gt queue is empty, it can become the new writable-by-wt queue.
                std::swap(begin_frame_being_drained_by_gt_, begin_frame_writable_by_wt_);

                resetBatchedWorkQueueReadyPromise(begin_frame_writable_by_wt_);

                // Signal to the game thread that we will be queuing additional BeginFrame work.
                SP_ASSERT(batched_work_queue_fifo_.size() < batched_work_queue_fifo_size_);
                SP_ASSERT(batched_work_queue_fifo_.empty() || batched_work_queue_fifo_.back().second == BatchedQueueType::EndFrame);
                batched_work_queue_fifo_.push_back(std::make_pair(begin_frame_writable_by_wt_, BatchedQueueType::BeginFrame));
            }

            // Push current work queue.
            SP_ASSERT(current_batched_work_queues_.empty());
            current_queuing_modes_.push_back(QueuingMode::Batched);
            current_batched_work_queues_.push_back(&batched_work_queues_.at(begin_frame_writable_by_wt_));

            return true; // return value should be interpreted as "inside critical section", not whether or not we're in an error state
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "execute_frame", [this]() -> bool {

            SP_ASSERT(!request_terminate_);

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("worker_thread: engine_service.execute_frame: Waiting for endFrame() to finish executing...");
            }

            // Wait until endFrame() on frame i-1 (or i-2 if we're in double-buffered mode) has finished
            // executing before we allow the user to start queuing work on the end_frame queue for frame i.
            batched_work_queue_ready_futures_.at(end_frame_being_drained_by_gt_).get();

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("worker_thread: engine_service.execute_frame: Finished waiting.");
            }

            // Reset promise and future and swap read/write indices.

            {
                std::lock_guard<std::mutex> lock(batched_work_queue_mutex_);

                // Now that the being-drained-by-gt queue is empty, it can become the new writable-by-wt queue.
                std::swap(end_frame_being_drained_by_gt_, end_frame_writable_by_wt_);

                resetBatchedWorkQueueReadyPromise(end_frame_writable_by_wt_);

                // Signal to the game thread that we will be queuing additional EndFrame work.
                SP_ASSERT(batched_work_queue_fifo_.size() < batched_work_queue_fifo_size_);
                SP_ASSERT(batched_work_queue_fifo_.empty() || batched_work_queue_fifo_.back().second == BatchedQueueType::BeginFrame);
                batched_work_queue_fifo_.push_back(std::make_pair(end_frame_writable_by_wt_, BatchedQueueType::EndFrame));
            }

            // Update current work queue.
            SP_ASSERT(!current_batched_work_queues_.empty() && current_batched_work_queues_.back() == &(batched_work_queues_.at(begin_frame_writable_by_wt_)));
            current_batched_work_queues_.back() = &(batched_work_queues_.at(end_frame_writable_by_wt_));

            // Allow beginFrame() to finish executing.

            std::string queue_name = batched_work_queues_.at(begin_frame_writable_by_wt_).getName();
            std::string func_name = "engine_service.execute_frame." + queue_name + ".reset";

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("worker_thread: engine_service.execute_frame: Scheduling ", func_name, " on work queue ", queue_name);
            }

            batched_work_queues_.at(begin_frame_writable_by_wt_).scheduleReset(func_name);

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("worker_thread: engine_service.execute_frame: Finished scheduling ", func_name);
            }

            return !request_error_; // return value should be interpreted as error state
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "end_frame", [this](bool single_step) -> bool {

            SP_ASSERT(!request_terminate_);

            // Only used in the single_step code path.
            bool request_begin_frame = !request_error_;

            if (single_step) {

                //
                // The code below is intentionally nearly identical to the implementation in execute_frame,
                // but with some minor differences. See comments below.
                //

                if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                    SP_LOG("worker_thread: engine_service.end_frame (single-step): Waiting for beginFrame() to finish executing...");
                }

                // Wait until beginFrame() on frame i-2 has finished executing before we allow the user to start
                // queuing work on the queue for frame i.
                batched_work_queue_ready_futures_.at(begin_frame_being_drained_by_gt_).get();

                if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                    SP_LOG("worker_thread: engine_service.end_frame (single-step): Finished waiting.");
                }

                // Reset promise and future and swap read/write indices.

                {
                    std::lock_guard<std::mutex> lock(batched_work_queue_mutex_);

                    // Now that the being-drained-by-gt queue is empty, it can become the new writable-by-wt queue.
                    std::swap(begin_frame_being_drained_by_gt_, begin_frame_writable_by_wt_);

                    // If we're not in an error state according to our local copy of the error state, then
                    // signal to the game thread that we will be queuing additional work. We only reset the promise
                    // and future if we push the queue to the FIFO, because otherwise the game thread will never
                    // set the promise, and releaseGameThread() would wait on the future forever.
                    if (request_begin_frame) {
                        resetBatchedWorkQueueReadyPromise(begin_frame_writable_by_wt_);

                        SP_ASSERT(batched_work_queue_fifo_.size() < batched_work_queue_fifo_size_);
                        SP_ASSERT(batched_work_queue_fifo_.empty() || batched_work_queue_fifo_.back().second == BatchedQueueType::EndFrame);
                        batched_work_queue_fifo_.push_back(std::make_pair(begin_frame_writable_by_wt_, BatchedQueueType::BeginFrame));
                    }
                }

                // Update current work queue.
                SP_ASSERT(!current_batched_work_queues_.empty() && current_batched_work_queues_.back() == &(batched_work_queues_.at(end_frame_writable_by_wt_)));
                if (request_begin_frame) {
                    current_batched_work_queues_.back() = &batched_work_queues_.at(begin_frame_writable_by_wt_);
                } else {
                    current_queuing_modes_.pop_back();
                    current_batched_work_queues_.pop_back();
                }

            } else {

                // Pop current work queue.
                SP_ASSERT(!current_batched_work_queues_.empty() && current_batched_work_queues_.back() == &(batched_work_queues_.at(end_frame_writable_by_wt_)));
                current_queuing_modes_.pop_back();
                current_batched_work_queues_.pop_back();
            }

            // Allow endFrame() to finish executing.

            std::string queue_name = batched_work_queues_.at(end_frame_writable_by_wt_).getName();
            std::string func_name = "engine_service.end_frame." + queue_name + ".reset";

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("worker_thread: engine_service.end_frame: Scheduling ", func_name, " on work queue ", queue_name);
            }

            batched_work_queues_.at(end_frame_writable_by_wt_).scheduleReset(func_name);

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("worker_thread: engine_service.end_frame: Finished scheduling ", func_name);
            }

            if (single_step) {
                return request_begin_frame; // return value should be interpreted as "inside critical section", not whether or not we're in an error state
            } else {
                return !request_error_; // return value should be interpreted as error state
            }
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "begin_editor_transaction", [this]() -> bool {

            SP_ASSERT(!request_terminate_);

            if (request_error_) {
                return false;
            }

            // Push current work queue.
            current_queuing_modes_.push_back(QueuingMode::Incremental);
            current_incremental_work_queues_.push_back(&editor_transaction_work_queue_);

            return true; // return value should be interpreted as "inside critical section", not whether or not we're in an error state
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "end_editor_transaction", [this]() -> bool {

            SP_ASSERT(!request_terminate_);

            // Pop current work queue.
            SP_ASSERT(!current_incremental_work_queues_.empty() && current_incremental_work_queues_.back() == &editor_transaction_work_queue_);
            current_queuing_modes_.pop_back();
            current_incremental_work_queues_.pop_back();

            return !request_error_; // return value should be interpreted as error state
        });

        bindFuncToExecuteOnGameThread("engine_service", "flush", []() -> void {});
    }

    static std::unique_ptr<EngineService<TEntryPointBinder>> create(TEntryPointBinder* entry_point_binder);

    template <typename TFunc>
    void bindFuncToExecuteOnWorkerThread(const std::string& service_name, const std::string& func_name, const TFunc& func)
    {
        std::string wt_thread_name = "worker_thread";
        std::string wt_long_func_name = service_name + ".call_sync_on_worker_thread." + func_name;
        bool wt_return_early_if_error = false;
        auto wt_func = wrapFuncToExecuteInTryCatch(wt_thread_name, wt_long_func_name, wt_return_early_if_error, func);

        SP_ASSERT(!func_signature_registry_.isFuncSignatureRegistered(wt_long_func_name));
        func_signature_registry_.registerFuncSignature(wt_long_func_name, func);
        entry_point_binder_->bind(wt_long_func_name, wt_func); 
    }

    template <typename TFunc>
    void bindFuncToExecuteOnGameThread(const std::string& service_name, const std::string& func_name, const TFunc& func)
    {
        UnrealEntryPointBindFlags bind_flags = UnrealEntryPointBindFlags::CallSync | UnrealEntryPointBindFlags::CallAsync | UnrealEntryPointBindFlags::SendAsync;
        bindFuncToExecuteOnGameThread(service_name, func_name, bind_flags, func);
    }

    template <typename TFunc>
    void bindFuncToExecuteOnGameThread(const std::string& service_name, const std::string& func_name, UnrealEntryPointBindFlags bind_flags, const TFunc& func)
    {
        bindFuncToExecuteOnGameThread(service_name, func_name, bind_flags, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs>
    void bindFuncToExecuteOnGameThread(const std::string& service_name, const std::string& func_name, UnrealEntryPointBindFlags bind_flags, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // Each gt_func lambda is typically executed on the game thread after it has been scheduled by the
        // worker thread.

        std::string gt_thread_name = "game_thread";
        bool gt_return_early_if_error = true;

        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::CallSync)) {
            std::string gt_call_sync_long_func_name = service_name + ".call_sync_on_game_thread." + func_name;
            auto gt_func = wrapFuncToExecuteInTryCatch(gt_thread_name, gt_call_sync_long_func_name, gt_return_early_if_error, func);
            SP_ASSERT(!func_signature_registry_.isFuncSignatureRegistered(gt_call_sync_long_func_name));
            func_signature_registry_.registerFuncSignature<TReturn, TArgs...>(gt_call_sync_long_func_name);
            entry_point_binder_->bind(gt_call_sync_long_func_name, wrapFuncToExecuteInWorkQueueBlocking(gt_call_sync_long_func_name, gt_func));
        }

        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::CallAsync)) {
            std::string gt_call_async_long_func_name = service_name + ".call_async_on_game_thread." + func_name;
            auto gt_func = wrapFuncToExecuteInTryCatch(gt_thread_name, gt_call_async_long_func_name, gt_return_early_if_error, func);
            SP_ASSERT(!func_signature_registry_.isFuncSignatureRegistered(gt_call_async_long_func_name));
            func_signature_registry_.registerFuncSignature<SpFuture, TArgs...>(gt_call_async_long_func_name);
            entry_point_binder_->bind(gt_call_async_long_func_name, wrapFuncToExecuteInWorkQueueNonBlocking(gt_call_async_long_func_name, gt_func)); // bound function calls createFuture

            // if we're binding a call_async function, then we also need to bind a corresponding
            // engine_service.get_future_result_as_return_type function if we haven't already

            std::string wt_thread_name = "worker_thread";
            std::string wt_get_future_result_long_func_name = "engine_service.get_future_result_from_game_thread_as_" + FuncSignatureRegistry::getTypeName<TReturn>();
            bool wt_return_early_if_error = false;
            auto wt_func = wrapFuncToExecuteInTryCatch(wt_thread_name, wt_get_future_result_long_func_name, wt_return_early_if_error,
                [this, wt_get_future_result_long_func_name](SpFuture& future) -> TReturn {

                    // If we're in an error state and the future isn't ready, then the corresponding work item
                    // might never execute, e.g., because its work queue will be skipped by beginFrame() or
                    // endFrame(), so calling get() could block forever. In this case, we return a
                    // default-constructed object and intentionally leak the future.
                    if (request_error_ && !isFutureReady<TReturn>(future)) {
                        SP_LOG("worker_thread: ERROR: In an error state when attempting to execute ", wt_get_future_result_long_func_name, " and the future isn't ready. Attempting to wait for the future could deadlock because its corresponding task might never execute. Intentionally leaking the future object and returning a default-constructed return value...");
                        return TReturn();
                    }

                    return destroyFuture<TReturn>(future); // bound function calls destroyFuture
                });

            if (!func_signature_registry_.isFuncSignatureRegistered(wt_get_future_result_long_func_name)) {
                func_signature_registry_.registerFuncSignature<TReturn, SpFuture&>(wt_get_future_result_long_func_name);
                entry_point_binder_->bind(wt_get_future_result_long_func_name, wt_func);
            }
        }

        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::SendAsync)) {
            std::string gt_send_async_long_func_name = service_name + ".send_async_on_game_thread." + func_name;
            auto gt_func = wrapFuncToExecuteInTryCatch(gt_thread_name, gt_send_async_long_func_name, gt_return_early_if_error, func);
            SP_ASSERT(!func_signature_registry_.isFuncSignatureRegistered(gt_send_async_long_func_name));
            func_signature_registry_.registerFuncSignature<void, TArgs...>(gt_send_async_long_func_name);
            entry_point_binder_->bind(gt_send_async_long_func_name, wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(gt_send_async_long_func_name, gt_func));
        }
    }

    // Called on the game thread from SpServices::requestInitialize() and SpServices::terminate()

    void startup()
    {
        if (Config::isInitialized()) {
            watchdog_sleep_time_seconds_ = Config::get<double>("SP_SERVICES.ENGINE_SERVICE.WATCHDOG_SLEEP_TIME_SECONDS");
            watchdog_max_time_seconds_ = Config::get<double>("SP_SERVICES.ENGINE_SERVICE.WATCHDOG_MAX_TIME_SECONDS");
        }

        watchdog_stop_requested_ = false;
        watchdog_thread_ = std::thread([this]() -> void { watchdog(); });
    }

    void shutdown()
    {
        // Request the watchdog thread to stop, and wake it up so it stops immediately rather than after its
        // current sleep time.

        {
            std::lock_guard<std::mutex> lock(watchdog_mutex_);
            watchdog_stop_requested_ = true;
        }
        watchdog_condition_variable_.notify_one();
        watchdog_thread_.join();

        // Prevent engine_service.initialize from re-initializing the work queues after we terminate them below.
        // engine_service.initialize checks shutdown_ while holding lifecycle_mutex_, so either it finishes
        // re-initializing the work queues before we acquire the lock, in which case we terminate them below, or it
        // observes that shutdown_ is true and returns without re-initializing them.

        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

        // Enter the error state, so any RPC handler that starts executing from now on returns immediately
        // rather than waiting on the game thread.

        request_error_ = true;

        // Release any RPC handler that is waiting in engine_service.begin_frame, engine_service.execute_frame,
        // or engine_service.end_frame. We set shutdown_ while holding both lifecycle_mutex_ and
        // batched_work_queue_mutex_, so it can be read while holding either one (see engine_service.initialize
        // and resetBatchedWorkQueueReadyPromise(...)).

        {
            std::lock_guard lock(batched_work_queue_mutex_);
            shutdown_ = true;
            for (int i = 0; i < NUM_BATCHED_QUEUES; i++) {
                if (!batched_work_queue_ready_promises_set_.at(i)) {
                    batched_work_queue_ready_promises_.at(i).set_value();
                    batched_work_queue_ready_promises_set_.at(i) = true;
                }
            }
        }

        // Release any RPC handler that is waiting on a task that will never execute, e.g., because the Python
        // client scheduled the task inside an instance.begin_frame() block, but the engine exited before the
        // game thread got a chance to execute it. Terminating a work queue destroys its pending tasks, and any
        // task scheduled after this point is discarded immediately (see WorkQueue::scheduleFunc(...)), so the
        // corresponding futures throw a broken-promise exception. This is safe because we're on the game
        // thread, so it isn't inside WorkQueue::run() or WorkQueue::pollOne().

        for (int i = 0; i < NUM_BATCHED_QUEUES; i++) {
            batched_work_queues_.at(i).terminate();
        }

        editor_transaction_work_queue_.terminate();
    }

protected:
    void beginFrame() override
    {
        Service::beginFrame();

        bool handle_frame = false;
        int readable_by_gt = -1;

        // Only handle the frame if the front of our FIFO is a queue of type BeginFrame.

        {
            std::lock_guard<std::mutex> lock(batched_work_queue_mutex_);
            if (batched_work_queue_fifo_.size() > 0) {
                auto [queue_id, queue_type] = batched_work_queue_fifo_.front(); // returns front without removing
                if (queue_type == BatchedQueueType::BeginFrame) {
                    handle_frame = true;
                    readable_by_gt = queue_id;
                    batched_work_queue_fifo_.pop_front(); // removes front without returning
                }
            }
        }

        if (request_terminate_ || request_error_) {
            handle_frame = false;
        }

        if (frame_state_ == FrameState::Terminating) {
            handle_frame = false;
        }

        if (handle_frame) {

            // Update frame state.
            frame_state_ = FrameState::ExecutingBeginFrame;

            std::string queue_name = batched_work_queues_.at(readable_by_gt).getName();

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("game_thread: beginFrame(): Executing ", queue_name, "...");
            }

            // Execute all pre-frame work and wait here until engine_service.execute_frame calls
            // scheduleReset(...) and the scheduled reset executes.
            batched_work_queues_.at(readable_by_gt).run();

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("game_thread: beginFrame(): Finished executing ", queue_name, ".");
            }

            frame_state_ = FrameState::ExecutingFrame;
        }

        // Allow engine_service.begin_frame (or engine_service.end_frame if we're in single-step mode) to finish
        // executing. If we popped a queue from the FIFO, set its promise even if we didn't handle the frame, e.g.,
        // because we're in an error state, so the worker thread can always proceed if it is waiting on the
        // corresponding future, e.g., in releaseGameThread().
        if (readable_by_gt != -1) {
            std::lock_guard<std::mutex> lock(batched_work_queue_mutex_);
            batched_work_queue_ready_promises_.at(readable_by_gt).set_value();
            batched_work_queue_ready_promises_set_.at(readable_by_gt) = true;
        }
    }

    void endFrame() override
    {
        bool handle_frame = false;
        int readable_by_gt = -1;

        // Only handle the frame if the front of our FIFO is a queue of type EndFrame.

        {
            std::lock_guard<std::mutex> lock(batched_work_queue_mutex_);
            if (batched_work_queue_fifo_.size() > 0) {
                auto [queue_id, queue_type] = batched_work_queue_fifo_.front(); // returns front without removing
                if (queue_type == BatchedQueueType::EndFrame) {
                    handle_frame = true;
                    readable_by_gt = queue_id;
                    batched_work_queue_fifo_.pop_front(); // removes front without returning
                }
            }
        }

        if (request_terminate_ || request_error_) {
            handle_frame = false;
        }

        if (frame_state_ == FrameState::Terminating) {
            handle_frame = false;
        }

        if (handle_frame) {

            // Update frame state.
            frame_state_ = FrameState::ExecutingEndFrame;

            std::string queue_name = batched_work_queues_.at(readable_by_gt).getName();

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("game_thread: endFrame(): Executing ", queue_name, "...");
            }

            // Execute all post-frame work and wait here until engine_service.end_frame calls
            // scheduleReset(...) and the scheduled reset executes.
            batched_work_queues_.at(readable_by_gt).run();

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("game_thread: endFrame(): Finished executing ", queue_name, ".");
            }

            // Update frame state.
            frame_state_ = FrameState::Idle;
        }

        // Allow engine_service.execute_frame to finish executing. If we popped a queue from the FIFO, set its
        // promise even if we didn't handle the frame, e.g., because we're in an error state, so the worker thread
        // can always proceed if it is waiting on the corresponding future, e.g., in releaseGameThread().
        if (readable_by_gt != -1) {
            std::lock_guard<std::mutex> lock(batched_work_queue_mutex_);
            batched_work_queue_ready_promises_.at(readable_by_gt).set_value();
            batched_work_queue_ready_promises_set_.at(readable_by_gt) = true;
        }

        Service::endFrame();
    }

public:
    void executeEditorPostTransaction()
    {
        bool handle_frame = true;

        if (request_error_) {
            handle_frame = false;
        }

        if (request_terminate_) {
            handle_frame = false;
        }

        if (frame_state_ == FrameState::Terminating) {
            handle_frame = false;
        }

        if (handle_frame) {

            std::string queue_name = editor_transaction_work_queue_.getName();

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("game_thread: executeEditorPostTransaction(): Executing ", queue_name, "...");
            }

            // Execute a single editor-transaction task.
            std::size_t num_tasks_executed = editor_transaction_work_queue_.pollOne();
            SP_ASSERT(num_tasks_executed == 1);

            if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("game_thread: executeEditorPostTransaction(): Finished executing ", queue_name, ".");
            }
        }
    }

private:
    enum class BatchedQueueType
    {
        BeginFrame = 0,
        EndFrame   = 1
    };

    enum class BatchedQueues
    {
        BeginFrameA      = 0,
        EndFrameA        = 1,
        BeginFrameB      = 2, // only used in double-buffered mode
        EndFrameB        = 3, // only used in double-buffered mode
        NumBatchedQueues = 4
    };

    enum class QueuingMode
    {
        Batched     = 0,
        Incremental = 1
    };

    enum class FrameState
    {
        Idle                = 0,
        ExecutingBeginFrame = 1,
        ExecutingFrame      = 2,
        ExecutingEndFrame   = 3,
        Terminating         = 4
    };

    // In the wrapFunc(...) functions below, we assume that the input function always accepts all arguments
    // by non-const reference. This will avoid unnecessary copying when we eventually call the input function.
    // We can't assume the input function accepts arguments by const reference, because the input function
    // might want to modify the arguments, e.g., when an input function resolves pointers to shared memory
    // for an input SpPackedArray& before forwarding it to an inner function.

    // Typically called from the game thread in EngineService::bindFuncToExecuteOnWorkerThread(...) and
    // EngineService::bindFuncToExecuteOnGameThread(...) to get an outer func that executes the given inner func
    // in a try-catch block.

    template <typename TFunc>
    auto wrapFuncToExecuteInTryCatch(const std::string& thread_name, const std::string& func_name, bool return_early_if_error, const TFunc& func)
    {
        return wrapFuncToExecuteInTryCatch(thread_name, func_name, return_early_if_error, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInTryCatch(const std::string& thread_name, const std::string& func_name, bool return_early_if_error, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // The lambda declared below is either bound to a specific RPC entry point and called from the worker
        // thread by the RPC server, or scheduled on a work queue and called from the game thread. Note that we
        // capture by value because we want to guarantee that our captured variables are still accessible after
        // wrapFuncToExecuteInTryCatch(...) returns.

        return [this, thread_name, func_name, return_early_if_error, func](TArgs&... args) -> TReturn {
            return executeFuncInTryCatch(thread_name, func_name, return_early_if_error, [&thread_name, &func_name, &func, &args...]() -> TReturn {
                if constexpr (std::is_void_v<TReturn>) {

                    if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG(thread_name, ": Executing function ", func_name, "...");
                    }

                    func(args...);

                    if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG(thread_name, ": Finished executing function ", func_name, ".");
                    }

                } else {

                    if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG(thread_name, ": Executing function ", func_name, "...");
                    }

                    TReturn return_value = func(args...);

                    if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG(thread_name, ": Finished executing function ", func_name, ".");
                    }

                    return return_value;

                }
            });
        };
    }

    // Typically called from the game thread in EngineService::bindFuncToExecuteOnGameThread(...) to get an
    // outer func that executes the given inner func on a work queue and blocks until the result is ready.

    template <typename TFunc>
    auto wrapFuncToExecuteInWorkQueueBlocking(const std::string& func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInWorkQueueBlocking(func_name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInWorkQueueBlocking(const std::string& func_name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // The lambda declared below is typically bound to a specific RPC entry point and called from the
        // worker thread by the RPC server. Note that we capture by value because we want
        // to guarantee that our captured variables are still accessible after wrapFuncToExecuteInWorkQueueBlocking(...)
        // returns.

        return [this, func_name, func](TArgs&... args) -> TReturn {

            std::string thread_name = "worker_thread";
            bool return_early_if_error = true;

            return executeFuncInTryCatch(thread_name, func_name, return_early_if_error, [this, &func_name, &func, &args...]() -> TReturn {

                WorkQueue* work_queue = getCurrentWorkQueue();
                if (!work_queue) {
                    SP_LOG("worker_thread: ERROR: Current work queue is null when attempting to execute function: ", func_name);
                    SP_ASSERT(false);
                }

                if constexpr (std::is_void_v<TReturn>) {
                    if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("worker_thread: Executing function ", func_name, " on work queue ", work_queue->getName(), " (blocking)...");
                    }

                    work_queue->scheduleFunc(func_name, func, args...).get();

                    if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("worker_thread: Finished executing function ", func_name, " on work queue ", work_queue->getName(), " (blocking).");
                    }

                } else {
                    if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("worker_thread: Executing function ", func_name, " on work queue ", work_queue->getName(), " (blocking)...");
                    }

                    TReturn return_value = work_queue->scheduleFunc(func_name, func, args...).get();

                    if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("worker_thread: Finished executing function ", func_name, " on work queue ", work_queue->getName(), " (blocking).");
                    }

                    return return_value;
                }
            });
        };
    }

    // Typically called from the game thread in EngineService::bindFuncToExecuteOnGameThread(...) to get an
    // outer func that executes the given inner func on a work queue and returns a future.

    template <typename TFunc>
    auto wrapFuncToExecuteInWorkQueueNonBlocking(const std::string& func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInWorkQueueNonBlocking(func_name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInWorkQueueNonBlocking(const std::string& func_name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // The lambda declared below is typically bound to a specific RPC entry point and called from the
        // worker thread by the RPC server. Note that we capture func_name and func by value because we want
        // to guarantee that func_name and func are still accessible after wrapFuncToExecuteInWorkQueueNonBlocking(...)
        // returns.

        return [this, func_name, func](TArgs&... args) -> SpFuture {

            std::string thread_name = "worker_thread";
            bool return_early_if_error = true;

            return executeFuncInTryCatch(thread_name, func_name, return_early_if_error, [this, &func_name, &func, &args...]() -> SpFuture {

                WorkQueue* work_queue = getCurrentWorkQueue();
                if (!work_queue) {
                    SP_LOG("worker_thread: ERROR: Current work queue is null when attempting to execute function: ", func_name);
                    SP_ASSERT(false);
                }

                if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("worker_thread: Scheduling function ", func_name, " on work queue ", work_queue->getName(), " (non-blocking)...");
                }

                SpFuture future = createFuture(work_queue->scheduleFunc(func_name, func, args...));

                if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("worker_thread: Finished scheduling function ", func_name, " on work queue ", work_queue->getName(), " (non-blocking).");
                }

                return future;
            });
        };
    }

    // Typically called from the game thread in EngineService::bindFuncToExecuteOnGameThread(...) to get an
    // outer func that executes the given inner func on a work queue and ignores the return value.

    template <typename TFunc>
    auto wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(const std::string& func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(func_name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(const std::string& func_name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // The lambda declared below is typically bound to a specific RPC entry point and called from the
        // worker thread by the RPC server. Note that we capture func_name and func by value because we want
        // to guarantee that func_name and func are still accessible after
        // wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(...) returns.

        return [this, func_name, func](TArgs&... args) -> void {

            std::string thread_name = "worker_thread";
            bool return_early_if_error = true;

            executeFuncInTryCatch(thread_name, func_name, return_early_if_error, [this, &func_name, &func, &args...]() -> void {

                WorkQueue* work_queue = getCurrentWorkQueue();
                if (!work_queue) {
                    SP_LOG("worker_thread: ERROR: Current work queue is null when attempting to execute function: ", func_name);
                    SP_ASSERT(false);
                }

                if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("worker_thread: Scheduling function ", func_name, " on work queue ", work_queue->getName(), " (non-blocking, discarding return value)...");
                }

                work_queue->scheduleFunc(func_name, func, args...);

                if (Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("worker_thread: Finished scheduling function ", func_name, " on work queue ", work_queue->getName(), " (non-blocking, discarding return value)...");
                }
            });
        };
    }

    // Called from the worker thread and the game thread to execute a function in a try-catch block.

    template <typename TFunc>
    auto executeFuncInTryCatch(const std::string& thread_name, const std::string& func_name, bool return_early_if_error, const TFunc& func)
    {
        using TReturn = std::invoke_result_t<TFunc>;

        if (return_early_if_error && request_error_) {
            SP_LOG(thread_name, ": ERROR: In an error state when attempting to execute ", func_name, ", returning a default-constructed return value...");
            return TReturn();
        }

        try {

            // Announce that a throw occurring anywhere inside func() will be gracefully handled by the catch
            // blocks below, so SP_ASSERT can choose to throw (rather than exit or crash) if this is the only
            // way to get a response, e.g., because there is no interactive terminal available. We declare this
            // as the first statement inside the try block (rather than outside it) so the scope's destructor
            // runs before the matching catch block's body starts running, since a new exception thrown from
            // inside that catch body would not be caught by this same try-catch. See AssertsAreAllowedToThrowScope
            // in SpCore/Assert.h.
            AssertsAreAllowedToThrowScope asserts_are_allowed_to_throw_scope;

            return func();

        } catch (const std::exception& e) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG(thread_name, ": ERROR: Caught exception when executing ", func_name, ": ", e.what());
        } catch (...) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG(thread_name, ": ERROR: Caught unknown exception when executing ", func_name, ".");
        }

        // If we get this far, an exception has occurred, so set the error state and return a
        // default-constructed object.

        request_error_ = true;

        return TReturn();
    }

    // Called from the worker thread in engine_service.begin_frame, engine_service.execute_frame, and
    // engine_service.end_frame (if in single-step mode) while holding batched_work_queue_mutex_, to create a new
    // promise and future for a queue that is about to be pushed to the FIFO. If shutdown() has already been
    // called, the game thread will never set the new promise, so we set it immediately. Otherwise, a subsequent
    // call to releaseGameThread() would wait on the corresponding future forever. shutdown() sets shutdown_ while
    // holding batched_work_queue_mutex_, so either we create the promise before shutdown() sets the unset
    // promises, or we observe that shutdown_ is true here.

    void resetBatchedWorkQueueReadyPromise(int queue_id)
    {
        batched_work_queue_ready_promises_.at(queue_id) = std::promise<void>();
        batched_work_queue_ready_futures_.at(queue_id) = batched_work_queue_ready_promises_.at(queue_id).get_future();
        batched_work_queue_ready_promises_set_.at(queue_id) = false;

        if (shutdown_) {
            batched_work_queue_ready_promises_.at(queue_id).set_value();
            batched_work_queue_ready_promises_set_.at(queue_id) = true;
        }
    }

    // Called from the worker thread at the start of engine_service.initialize and engine_service.terminate, so it's
    // safe for those entry points to re-initialize or terminate the batched work queues, promises, and futures. If a
    // previous client exited in the middle of a transaction, the game thread might be stalled in run() waiting for a
    // reset that will never arrive. So we put the system into the same state as if an exception had occurred (as in
    // watchdog()), schedule a reset on every queue whose ready promise hasn't been set, and wait until the game
    // thread has set each of those promises. The game thread sets a queue's promise after it finishes executing the
    // queue, or after it pops the queue from the FIFO and skips it because we're in an error state.
    //
    // Only the worker thread clears a queue's ready promise flag, so the set of unready queues can only shrink after
    // we read it, and we don't need to hold batched_work_queue_mutex_ while scheduling resets. If a queue becomes
    // ready before its scheduled reset executes, the reset is discarded when the queue is re-initialized or
    // terminated. Each future we wait on is valid, because whenever the worker thread clears a queue's ready promise
    // flag, it also creates a new promise and future for the queue.

    void releaseGameThread()
    {
        request_error_ = true;

        std::vector<int> unready_queue_ids;
        {
            std::lock_guard<std::mutex> lock(batched_work_queue_mutex_);
            for (int i = 0; i < NUM_BATCHED_QUEUES; i++) {
                if (!batched_work_queue_ready_promises_set_.at(i)) {
                    unready_queue_ids.push_back(i);
                }
            }
        }

        for (int i : unready_queue_ids) {
            std::string func_name = "engine_service.release_game_thread." + batched_work_queues_.at(i).getName() + ".reset";
            batched_work_queues_.at(i).scheduleReset(func_name);
        }

        for (int i : unready_queue_ids) {
            batched_work_queue_ready_futures_.at(i).wait();
        }
    }

    // Functions for creating and destroying futures

    template <typename TReturn>
    static SpFuture createFuture(std::future<TReturn>&& std_future)
    {
        SpFuture future;
        future.future_ptr_ = new std::future<TReturn>(std::move(std_future));
        future.type_id_ = Std::getTypeIdString<TReturn>();
        SP_ASSERT(future.future_ptr_);
        return future;
    }

    template <typename TReturn>
    static bool isFutureReady(const SpFuture& future)
    {
        if (future.future_ptr_) {
            SP_ASSERT(future.type_id_ == Std::getTypeIdString<TReturn>());
            std::future<TReturn>* future_ptr = static_cast<std::future<TReturn>*>(future.future_ptr_);
            return future_ptr->wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        } else {
            return true; // destroyFuture(...) handles default-constructed SpFuture objects without blocking
        }
    }

    template <typename TReturn>
    static TReturn destroyFuture(SpFuture& future)
    {
        if (future.future_ptr_) {
            SP_ASSERT(future.type_id_ == Std::getTypeIdString<TReturn>());
            std::future<TReturn>* future_ptr = static_cast<std::future<TReturn>*>(future.future_ptr_);
            TReturn result = future_ptr->get();
            delete future_ptr;
            future.future_ptr_ = nullptr;
            return result;
        } else {
            SP_LOG("ERROR: Attempting to destroy a default-constructed SpFuture, returning a default-constructed return value...");
            return TReturn();
        }
    }

    template <>
    void destroyFuture<void>(SpFuture& future)
    {
        if (future.future_ptr_) {
            SP_ASSERT(future.type_id_ == Std::getTypeIdString<void>());
            std::future<void>* future_ptr = static_cast<std::future<void>*>(future.future_ptr_);
            future_ptr->get();
            delete future_ptr;
            future.future_ptr_ = nullptr;
        } else {
            SP_LOG("ERROR: Attempting to destroy a default-constructed SpFuture, returning...");
        }
    }

    WorkQueue* getCurrentWorkQueue() const
    {
        SP_ASSERT(current_queuing_modes_.size() == current_batched_work_queues_.size() + current_incremental_work_queues_.size());
        SP_ASSERT(!current_queuing_modes_.empty());

        switch (current_queuing_modes_.back()) {
            case QueuingMode::Batched:
                SP_ASSERT(!current_batched_work_queues_.empty());
                return current_batched_work_queues_.back();
            case QueuingMode::Incremental:
                SP_ASSERT(!current_incremental_work_queues_.empty());
                return current_incremental_work_queues_.back();
            default:
                SP_ASSERT(false);
                return nullptr;
        }
    }

    // Executes on the watchdog thread. Periodically checks if the game thread has been waiting for work in a
    // batched work queue's run() for longer than WATCHDOG_MAX_TIME_SECONDS, e.g., because the user's Python code
    // terminated in the middle of a transaction. Time spent executing work items doesn't count, because each
    // queue's idle ID is 0 while a work item is executing, and changes whenever a work item finishes executing.
    // The watchdog only needs each queue's idle ID, so it keeps track of when each idle ID last changed locally.
    // If a queue has been stalled for too long, we set the error state and reset the stalled queue, mirroring
    // what we do when an exception is thrown in executeFuncInTryCatch(...). We set the error state before
    // resetting the queue, so beginFrame() and endFrame() will discard any remaining entries in the FIFO rather
    // than stalling the game thread again.

    void watchdog()
    {
        std::array<uint64_t, NUM_BATCHED_QUEUES> last_idle_ids;
        std::array<std::chrono::steady_clock::time_point, NUM_BATCHED_QUEUES> last_idle_id_change_times;
        last_idle_ids.fill(0);
        last_idle_id_change_times.fill(std::chrono::steady_clock::now());

        std::chrono::duration<double> sleep_time(watchdog_sleep_time_seconds_);

        std::unique_lock<std::mutex> watchdog_lock(watchdog_mutex_);
        while (!watchdog_condition_variable_.wait_for(watchdog_lock, sleep_time, [this]() -> bool { return watchdog_stop_requested_; })) {
            std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            for (int i = 0; i < NUM_BATCHED_QUEUES; i++) {
                uint64_t idle_id = batched_work_queues_.at(i).getIdleId();
                if (idle_id != last_idle_ids.at(i)) {
                    last_idle_ids.at(i) = idle_id;
                    last_idle_id_change_times.at(i) = now;
                }
                if (idle_id != 0) {
                    double seconds = std::chrono::duration<double>(now - last_idle_id_change_times.at(i)).count();
                    if (seconds > watchdog_max_time_seconds_) {
                        std::string queue_name = batched_work_queues_.at(i).getName();
                        #if BOOST_COMP_MSVC
                            std::string seconds_str = std::format("{:.2f}", seconds);
                        #elif BOOST_COMP_CLANG
                            std::string seconds_str = (boost::format("%.2f")%seconds).str();
                        #else
                            #error
                        #endif
                        SP_LOG("watchdog_thread: ERROR: The game thread has been stalled for ", seconds_str, " seconds while executing the work queue \"", queue_name, "\". No new work has been scheduled on the queue in that time, and it has also not received a signal indicating that it is allowed to proceed. Attempting to un-stall the game thread by calling scheduleReset() on the queue...");
                        request_error_ = true;
                        batched_work_queues_.at(i).scheduleReset("engine_service.watchdog." + queue_name + ".reset");
                        last_idle_id_change_times.at(i) = now;
                    }
                }
            }
        }
    }

    FuncSignatureRegistry func_signature_registry_;
    TEntryPointBinder* entry_point_binder_ = nullptr;

    // Batched work queue state.
    //
    // Batched work queues are safe to access on multiple threads, except when they are initialized or
    // terminated (see engine_service.initialize and engine_service.terminate). The mutex is used to coordinate
    // access to the FIFO, promises, futures, and bools defined below.

    static const int BATCHED_WORK_QUEUE_FIFO_SIZE_SINGLE_BUFFERED = 2;
    static const int BATCHED_WORK_QUEUE_FIFO_SIZE_DOUBLE_BUFFERED = 4;
    static const int NUM_BATCHED_QUEUES                           = static_cast<int>(BatchedQueues::NumBatchedQueues);

    std::array<WorkQueue, NUM_BATCHED_QUEUES> batched_work_queues_ = {
        WorkQueue("begin_frame_work_queue_A"),
        WorkQueue("end_frame_work_queue_A"),
        WorkQueue("begin_frame_work_queue_B"), // only used in double-buffered mode
        WorkQueue("end_frame_work_queue_B")};  // only used in double-buffered mode

    std::mutex batched_work_queue_mutex_;

    // Only accessed on the worker thread.
    int begin_frame_writable_by_wt_      = static_cast<int>(BatchedQueues::BeginFrameA);
    int begin_frame_being_drained_by_gt_ = static_cast<int>(BatchedQueues::BeginFrameA); // set to BeginFrameB in engine_service.initialize in double-buffered mode

    // Only accessed on the worker thread.
    int end_frame_writable_by_wt_      = static_cast<int>(BatchedQueues::EndFrameA);
    int end_frame_being_drained_by_gt_ = static_cast<int>(BatchedQueues::EndFrameA); // set to EndFrameB in engine_service.initialize in double-buffered mode

    int batched_work_queue_fifo_size_ = BATCHED_WORK_QUEUE_FIFO_SIZE_SINGLE_BUFFERED; // set to BATCHED_WORK_QUEUE_FIFO_SIZE_DOUBLE_BUFFERED in double-buffered

    // This FIFO is written to by the worker thread in engine_service.begin_frame, engine_service.execute_frame,
    // and engine_service.end_frame (if in single-step mode). The FIFO is drained by the game thread in
    // beginFrame() and endFrame().
    boost::circular_buffer<std::pair<int, BatchedQueueType>> batched_work_queue_fifo_ = boost::circular_buffer<std::pair<int, BatchedQueueType>>(NUM_BATCHED_QUEUES);

    // These promises and futures are each initialized by the worker thread in engine_service.initialize,
    // engine_service.begin_frame, engine_service.execute_frame, and engine_service.end_frame (if in single-step
    // mode). Subsequently, the promises are only ever set by the game thread in beginFrame(), endFrame(), and
    // shutdown(). The futures are read on the worker thread in engine_service.begin_frame,
    // engine_service.execute_frame, engine_service.end_frame (if in single-step mode), and releaseGameThread().
    // Each bool variable is set to true by the game thread whenever the corresponding promise is set, and set to
    // false by the worker thread whenever the corresponding promise is re-initialized. The bools are read by the
    // worker thread in releaseGameThread() to determine which queues the game thread might be stalled on, and
    // by the game thread in shutdown() to avoid deadlocks when shutting down the application.
    std::array<std::promise<void>, NUM_BATCHED_QUEUES> batched_work_queue_ready_promises_;
    std::array<std::future<void>,  NUM_BATCHED_QUEUES> batched_work_queue_ready_futures_;
    std::array<std::atomic<bool>,  NUM_BATCHED_QUEUES> batched_work_queue_ready_promises_set_ = {true, true, true, true};

    // Incremental work queue state.

    // The incremental work queue is safe to access on multiple threads, except when it is initialized or
    // terminated. Work is scheduled on the worker
    // thread in engine_service.begin_editor_transaction and engine_service.end_editor_transaction. Work is
    // drained by the game thread in executeEditorPostTransaction().
    WorkQueue editor_transaction_work_queue_ = WorkQueue("editor_transaction_work_queue");

    // Current work queue stacks.

    // Only accessed on the worker thread. The queuing mode stack tracks which type of work queue is active;
    // its size is always equal to the sum of the sizes of the two pointer stacks. The pointer stacks track
    // the active batched and incremental work queues respectively.
    std::vector<QueuingMode> current_queuing_modes_;
    std::vector<WorkQueue*> current_batched_work_queues_;
    std::vector<WorkQueue*> current_incremental_work_queues_;

    // Written and read by the game thread in beginFrame() and endFrame(). Written by the worker thread in
    // engine_service.initialize and engine_service.terminate. Read by the game thread in
    // executeEditorPostTransaction().
    std::atomic<FrameState> frame_state_ = FrameState::Idle;

    // Written by the worker thread in engine_service.initialize and engine_service.terminate to indicate to
    // the game thread that we want to terminate the application. Read by the worker thread in
    // engine_service.begin_frame, engine_service.execute_frame, engine_service.end_frame, and
    // engine_service.begin_editor_transaction, engine_service.end_editor_transaction. Read by the game thread
    // in beginFrame(), endFrame(), and executeEditorPostTransaction().
    std::atomic<bool> request_terminate_ = false;

    // Written by the worker thread in engine_service.initialize. Written by the worker thread and the game
    // thread in executeFuncInTryCatch(...), by the watchdog thread in watchdog(), and by the game thread in
    // shutdown(), to indicate that an error has occurred. Read by the worker thread in engine_service.begin_frame,
    // engine_service.execute_frame, engine_service.end_frame, and engine_service.begin_editor_transaction,
    // engine_service.end_editor_transaction. Read by the game thread in beginFrame(), endFrame(), and
    // executeEditorPostTransaction(). Read by the worker thread and the game thread in executeFuncInTryCatch(...).
    std::atomic<bool> request_error_ = false;

    // Lifecycle state.

    // The mutex is held by the worker thread in engine_service.initialize and engine_service.terminate while it
    // re-initializes or terminates the work queues, by the game thread in shutdown() while it terminates the
    // work queues, and by the watchdog thread while it checks and resets the batched work queues. shutdown_ is
    // set to true by the game thread in shutdown() while holding both lifecycle_mutex_ and
    // batched_work_queue_mutex_. It is read by the worker thread in engine_service.initialize while holding
    // lifecycle_mutex_, and in resetBatchedWorkQueueReadyPromise(...) while holding batched_work_queue_mutex_.
    std::mutex lifecycle_mutex_;
    bool shutdown_ = false;

    // Watchdog state.

    // The watchdog thread is started in startup() and stopped in shutdown(), which are both called on the game
    // thread. The mutex and condition variable are used to wake up the watchdog thread immediately when
    // shutdown() requests it to stop. If the watchdog thread needs to hold both mutexes, it acquires
    // watchdog_mutex_ before lifecycle_mutex_.
    std::thread watchdog_thread_;
    std::mutex watchdog_mutex_;
    std::condition_variable watchdog_condition_variable_;
    bool watchdog_stop_requested_ = false;
    double watchdog_sleep_time_seconds_ = 1.0;
    double watchdog_max_time_seconds_ = 10.0;
};
