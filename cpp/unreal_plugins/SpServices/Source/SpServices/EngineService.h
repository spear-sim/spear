//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // int64_t, uint64_t

#include <algorithm>   // std::swap
#include <array>
#include <atomic>
#include <exception>
#include <future>      // std::promise
#include <map>
#include <mutex>       // std::lock_guard
#include <ranges>      // std::views::transform
#include <string>
#include <utility>     // std::make_pair, std::move, std::pair
#include <type_traits> // std::invoke_result_t, std::is_void_v
#include <vector>

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

template <CEntryPointBinder TEntryPointBinder>
class EngineService : public Service
{
public:
    EngineService() = delete;
    EngineService(TEntryPointBinder* entry_point_binder) : Service("EngineService")
    {
        SP_ASSERT(entry_point_binder);

        entry_point_signature_registries_["call_sync_on_worker_thread"] = FuncSignatureRegistry();

        entry_point_signature_registries_["call_sync_on_game_thread"]           = FuncSignatureRegistry();
        entry_point_signature_registries_["call_async_on_game_thread"]          = FuncSignatureRegistry();
        entry_point_signature_registries_["send_async_on_game_thread"]          = FuncSignatureRegistry();
        entry_point_signature_registries_["get_future_result_from_game_thread"] = FuncSignatureRegistry();

        entry_point_binder_ = entry_point_binder;

        //
        // Entry points for initializing and terminating the frame state.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "initialize", [this]() -> void {

            // Reset frame state.
            frame_state_ = FrameState::Idle;

            // Reset request flags.
            request_terminate_ = false;
            request_error_ = false;

            // Reset state for queue management.

            bool double_buffered_work_queues = false;
            if (Config::isInitialized()) {
                double_buffered_work_queues = Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.DOUBLE_BUFFERED_WORK_QUEUES");
            }

            if (double_buffered_work_queues) {
                begin_frame_writable_by_wt_      = 0;
                end_frame_writable_by_wt_        = 1;
                begin_frame_being_drained_by_gt_ = 2;
                end_frame_being_drained_by_gt_   = 3;
            } else {
                begin_frame_writable_by_wt_      = 0;
                end_frame_writable_by_wt_        = 1;
                begin_frame_being_drained_by_gt_ = 0;
                end_frame_being_drained_by_gt_   = 1;
            }

            // There is no need to lock because it is safe to access WorkQueue objects from multiple threads.
            for (int i = 0; i < 4; i++) {
                work_queues_.at(i).initialize();
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                for (int i = 0; i < 4; i++) {
                    work_queue_ready_promises_.at(i) = std::promise<void>();
                    work_queue_ready_futures_.at(i) = work_queue_ready_promises_.at(i).get_future();
                    work_queue_ready_promises_.at(i).set_value();
                    work_queue_ready_promises_set_.at(i) = true;
                }
            }
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "terminate", [this]() -> void {

            // Set the frame state to prevent calling any more entry points.
            frame_state_ = FrameState::Terminating;

            // There is no need to lock because it is safe to access WorkQueue objects from multiple threads.
            for (int i = 0; i < 4; i++) {
                work_queues_.at(i).terminate();
            }

            // Indicate to the game thread that we want to terminate the application.
            request_terminate_ = true;
        });

        //
        // Entry points for validating other entry points.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "get_entry_point_signature_type_descs", [this]() -> std::vector<SpFuncSignatureTypeDesc> {
            return FuncSignatureRegistry::getFuncSignatureTypeDescs();
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "get_entry_point_signature_descs", [this]() -> std::map<std::string, std::vector<SpFuncSignatureDesc>> {
            return Std::toMap<std::string, std::vector<SpFuncSignatureDesc>>(
                entry_point_signature_registries_ |
                std::views::transform([](auto& pair) {
                    auto& [entry_point_registry_name, entry_point_registry] = pair;
                    return std::make_pair(entry_point_registry_name, entry_point_registry.getFuncSignatureDescs());
                }));
        });

        //
        // Entry points for managing the frame state.
        //

        bindFuncToExecuteOnWorkerThread("engine_service", "begin_frame", [this]() -> bool {

            SP_ASSERT(!request_terminate_);

            if (request_error_) {
                return false;
            }

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.begin_frame: Waiting for beginFrame() to finish executing...");
            }

            // Wait until beginFrame() on frame i-1 (or i-2 if we're in double-buffered mode) has finished
            // executing before we allow the user to start queuing work on the queue for frame i.
            work_queue_ready_futures_.at(begin_frame_being_drained_by_gt_).get();

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.begin_frame: Finished waiting.");
            }

            // Reset promise and future and swap read/write indices.

            {
                std::lock_guard<std::mutex> lock(mutex_);

                // Now that the being-drained-by-gt queue is empty, it can become the new writable-by-wt queue. 
                std::swap(begin_frame_being_drained_by_gt_, begin_frame_writable_by_wt_);

                work_queue_ready_promises_.at(begin_frame_writable_by_wt_) = std::promise<void>();
                work_queue_ready_futures_.at(begin_frame_writable_by_wt_) = work_queue_ready_promises_.at(begin_frame_writable_by_wt_).get_future();
                work_queue_ready_promises_set_.at(begin_frame_writable_by_wt_) = false;

                // Signal to the game thread that we will be queuing additional BeginFrame work.
                SP_ASSERT(!work_queue_fifo_.full());
                SP_ASSERT(work_queue_fifo_.empty() || work_queue_fifo_.back().second == QueueType::EndFrame);
                work_queue_fifo_.push_back(std::make_pair(begin_frame_writable_by_wt_, QueueType::BeginFrame));
            }

            // Set current work queue.
            SP_ASSERT(current_work_queue_ == nullptr);
            current_work_queue_ = &work_queues_.at(begin_frame_writable_by_wt_);

            return true; // return value should be interpreted as "inside critical section", not whether or not we're in an error state
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "execute_frame", [this]() -> bool {

            SP_ASSERT(!request_terminate_);

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.execute_frame: Waiting for endFrame() to finish executing...");
            }

            // Wait until endFrame() on frame i-1 (or i-2 if we're in double-buffered mode) has finished
            // executing before we allow the user to start queuing work on the end_frame queue for frame i.
            work_queue_ready_futures_.at(end_frame_being_drained_by_gt_).get();

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.execute_frame: Finished waiting.");
            }

            // Reset promise and future and swap read/write indices.

            {
                std::lock_guard<std::mutex> lock(mutex_);

                // Now that the being-drained-by-gt queue is empty, it can become the new writable-by-wt queue. 
                std::swap(end_frame_being_drained_by_gt_, end_frame_writable_by_wt_);

                work_queue_ready_promises_.at(end_frame_writable_by_wt_) = std::promise<void>();
                work_queue_ready_futures_.at(end_frame_writable_by_wt_)  = work_queue_ready_promises_.at(end_frame_writable_by_wt_).get_future();
                work_queue_ready_promises_set_.at(end_frame_writable_by_wt_) = false;

                // Signal to the game thread that we will be queuing additional EndFrame work.
                SP_ASSERT(!work_queue_fifo_.full());
                SP_ASSERT(work_queue_fifo_.empty() || work_queue_fifo_.back().second == QueueType::BeginFrame);
                work_queue_fifo_.push_back(std::make_pair(end_frame_writable_by_wt_, QueueType::EndFrame));
            }

            // Set current work queue.
            SP_ASSERT(current_work_queue_ == &(work_queues_.at(begin_frame_writable_by_wt_)));
            current_work_queue_ = &(work_queues_.at(end_frame_writable_by_wt_));

            // Allow beginFrame() to finish executing.

            std::string queue_name = work_queues_.at(begin_frame_writable_by_wt_).getName();
            std::string func_name = "engine_service.execute_frame." + queue_name + ".reset";

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.execute_frame: Scheduling ", func_name, " on work queue ", queue_name);
            }

            int writable_by_gt = begin_frame_writable_by_wt_;
            work_queues_.at(writable_by_gt).scheduleFunc(func_name, [this, writable_by_gt]() -> void {
                work_queues_.at(writable_by_gt).reset();
            });

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.execute_frame: Finished scheduling ", func_name);
            }

            return !request_error_; // return value should be interpreted as error state
        });

        bindFuncToExecuteOnWorkerThread("engine_service", "end_frame", [this](bool single_step) -> bool {

            SP_ASSERT(!request_terminate_);

            bool request_begin_frame = !request_error_;

            if (single_step) {

                //
                // The code below is intentionally nearly identical to the implementation in execute_frame,
                // but with some minor differences. See comments below.
                //

                if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                    SP_LOG("engine_service.end_frame (single-step): Waiting for beginFrame() to finish executing...");
                }

                // Wait until beginFrame() on frame i-2 has finished executing before we allow the user to start
                // queuing work on the queue for frame i.
                work_queue_ready_futures_.at(begin_frame_being_drained_by_gt_).get();

                if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                    SP_LOG("engine_service.end_frame (single-step): Finished waiting.");
                }

                // Reset promise and future and swap read/write indices.

                {
                    std::lock_guard<std::mutex> lock(mutex_);

                    // Now that the being-drained-by-gt queue is empty, it can become the new writable-by-wt queue. 
                    std::swap(begin_frame_being_drained_by_gt_, begin_frame_writable_by_wt_);

                    work_queue_ready_promises_.at(begin_frame_writable_by_wt_) = std::promise<void>();
                    work_queue_ready_futures_.at(begin_frame_writable_by_wt_) = work_queue_ready_promises_.at(begin_frame_writable_by_wt_).get_future();
                    work_queue_ready_promises_set_.at(begin_frame_writable_by_wt_) = false;

                    // If we're not in an error state according to our local copy of the error state, then
                    // signal to the game thread that we will be queuing additional work.
                    if (request_begin_frame) {
                        SP_ASSERT(!work_queue_fifo_.full());
                        work_queue_fifo_.push_back(std::make_pair(begin_frame_writable_by_wt_, QueueType::BeginFrame));
                    }
                }

                // Set current work queue.
                SP_ASSERT(current_work_queue_ == &(work_queues_.at(end_frame_writable_by_wt_)));
                if (request_begin_frame) {
                    current_work_queue_ = &work_queues_.at(begin_frame_writable_by_wt_);
                } else {
                    current_work_queue_ = nullptr;
                }

            } else {

                // Set current work queue.
                SP_ASSERT(current_work_queue_ == &(work_queues_.at(end_frame_writable_by_wt_)));
                current_work_queue_ = nullptr;
            }

            // Allow endFrame() to finish executing.

            std::string queue_name = work_queues_.at(end_frame_writable_by_wt_).getName();
            std::string func_name = "engine_service.end_frame." + queue_name + ".reset";

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.end_frame: Scheduling ", func_name, " on work queue ", queue_name);
            }

            int writable_by_gt = end_frame_writable_by_wt_;
            work_queues_.at(writable_by_gt).scheduleFunc(func_name, [this, writable_by_gt]() -> void {
                work_queues_.at(writable_by_gt).reset();
            });

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("engine_service.end_frame: Finished scheduling ", func_name);
            }

            if (single_step) {
                return request_begin_frame; // return value should be interpreted as "inside critical section", not whether or not we're in an error state
            } else {
                return !request_error_; // return value should be interpreted as error state
            }
        });
    }

    template <typename TFunc>
    void bindFuncToExecuteOnWorkerThread(const std::string& service_name, const std::string& func_name, const TFunc& func)
    {
        std::string long_func_name = service_name + ".call_sync_on_worker_thread." + func_name;
        FuncSignatureRegistry& entry_point_registry = entry_point_signature_registries_.at("call_sync_on_worker_thread");

        SP_ASSERT(!entry_point_registry.isFuncSignatureRegistered(long_func_name, func));
        entry_point_registry.registerFuncSignature(long_func_name, func);
        entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInTryCatch(long_func_name, func));
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
        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::CallSync)) {
            std::string long_func_name = service_name + ".call_sync_on_game_thread." + func_name;
            FuncSignatureRegistry& entry_point_registry = entry_point_signature_registries_.at("call_sync_on_game_thread");

            SP_ASSERT((!entry_point_registry.isFuncSignatureRegistered<TReturn, TArgs...>(long_func_name))); // extra parentheses needed because of comma
            entry_point_registry.registerFuncSignature<TReturn, TArgs...>(long_func_name);
            entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInWorkQueueBlocking(long_func_name, func));
        }

        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::CallAsync)) {
            std::string call_async_long_func_name = service_name + ".call_async_on_game_thread." + func_name;
            FuncSignatureRegistry& call_async_entry_point_registry = entry_point_signature_registries_.at("call_async_on_game_thread");

            SP_ASSERT((!call_async_entry_point_registry.isFuncSignatureRegistered<SpFuture, TArgs...>(call_async_long_func_name))); // extra parentheses needed because of comma
            call_async_entry_point_registry.registerFuncSignature<SpFuture, TArgs...>(call_async_long_func_name);
            entry_point_binder_->bind(call_async_long_func_name, wrapFuncToExecuteInWorkQueueNonBlocking(call_async_long_func_name, func)); // bound function calls createFuture

            // if we're binding a call_async function, then we also need to bind a corresponding
            // engine_service.get_future_result_as_return_type function if we haven't already

            std::string get_future_result_long_func_name = "engine_service.get_future_result_from_game_thread_as_" + FuncSignatureRegistry::getFuncSignatureTypeDesc<TReturn>().type_names_.at("entry_point");
            FuncSignatureRegistry& get_future_result_entry_point_registry = entry_point_signature_registries_.at("get_future_result_from_game_thread");

            if (!get_future_result_entry_point_registry.isFuncSignatureRegistered<TReturn, SpFuture&>(get_future_result_long_func_name)) {
                get_future_result_entry_point_registry.registerFuncSignature<TReturn, SpFuture&>(get_future_result_long_func_name);
                entry_point_binder_->bind(get_future_result_long_func_name, wrapFuncToExecuteInTryCatch(get_future_result_long_func_name, [this](SpFuture& future) -> TReturn {
                    return destroyFuture<TReturn>(future); // bound function calls destroyFuture
                }));
            }
        }

        if (Std::toBool(bind_flags & UnrealEntryPointBindFlags::SendAsync)) {
            std::string long_func_name = service_name + ".send_async_on_game_thread." + func_name;
            FuncSignatureRegistry& entry_point_registry = entry_point_signature_registries_.at("send_async_on_game_thread");

            SP_ASSERT((!entry_point_registry.isFuncSignatureRegistered<void, TArgs...>(long_func_name))); // extra parentheses needed because of comma
            entry_point_registry.registerFuncSignature<void, TArgs...>(long_func_name);
            entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(long_func_name, func));
        }
    }

    // Called on the game thread from SpServices::StartupModule() and SpServices::ShutdownModule()

    void startup() const {}

    void shutdown()
    {
        {
            std::lock_guard lock(mutex_);
            for (int i = 0; i < 4; i++) {
                if (!work_queue_ready_promises_set_.at(i)) {
                    work_queue_ready_promises_.at(i).set_value();
                    work_queue_ready_promises_set_.at(i) = true;
                }
            }
        }
    }

protected:
    void beginFrame() override
    {
        Service::beginFrame();

        bool handle_frame = false;
        int readable_by_gt = -1;

        // Only handle the frame if the front of our FIFO is a queue of type BeginFrame.

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (work_queue_fifo_.size() > 0) {
                auto [queue_id, queue_type] = work_queue_fifo_.front(); // returns front without removing
                if (queue_type == QueueType::BeginFrame) {
                    handle_frame = true;
                    readable_by_gt = queue_id;
                    work_queue_fifo_.pop_front(); // removes front without returning
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

            std::string queue_name = work_queues_.at(readable_by_gt).getName();
            std::string func_name = queue_name + ".run";

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("beginFrame(): Executing ", queue_name, "...");
            }

            // Execute all pre-frame work and wait here until engine_service.execute_frame queues a call to
            // reset() and the call executes.
            executeFuncInTryCatch(func_name, [this, readable_by_gt]() -> void {
                work_queues_.at(readable_by_gt).run();
            });

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("beginFrame(): Finished executing ", queue_name, ".");
            }

            frame_state_ = FrameState::ExecutingFrame;

            // Allow engine_service.begin_frame (or engine_service.end_frame if we're in single-step mode) to finish executing.

            {
                std::lock_guard<std::mutex> lock(mutex_);
                work_queue_ready_promises_.at(readable_by_gt).set_value();
                work_queue_ready_promises_set_.at(readable_by_gt) = true;
            }
        }
    }

    void endFrame() override
    {
        bool handle_frame = false;
        int readable_by_gt = -1;

        // Only handle the frame if the front of our FIFO is a queue of type EndFrame.

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (work_queue_fifo_.size() > 0) {
                auto [queue_id, queue_type] = work_queue_fifo_.front(); // returns front without removing
                if (queue_type == QueueType::EndFrame) {
                    handle_frame = true;
                    readable_by_gt = queue_id;
                    work_queue_fifo_.pop_front(); // removes front without returning
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

            std::string queue_name = work_queues_.at(readable_by_gt).getName();
            std::string func_name = queue_name + ".run";

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("endFrame(): Executing ", queue_name, "...");
            }

            // Execute all post-frame work and wait here until engine_service.end_frame queues a call to
            // reset() and the call executes.
            executeFuncInTryCatch(func_name, [this, readable_by_gt]() -> void {
                work_queues_.at(readable_by_gt).run();
            });

            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_FRAME_DEBUG_INFO")) {
                SP_LOG("endFrame(): Finished executing ", queue_name, ".");
            }

            // Update frame state.
            frame_state_ = FrameState::Idle;

            // Allow engine_service.execute_frame to finish executing.

            {
                std::lock_guard<std::mutex> lock(mutex_);
                work_queue_ready_promises_.at(readable_by_gt).set_value();
                work_queue_ready_promises_set_.at(readable_by_gt) = true;
            }
        }

        Service::endFrame();
    }

private:

    enum class QueueType
    {
        BeginFrame = 0,
        EndFrame   = 1
    };

    enum class FrameState
    {
        Idle                = 0,
        ExecutingBeginFrame = 1,
        ExecutingFrame      = 2,
        ExecutingEndFrame   = 3,
        Terminating         = 4,
        Error               = 5
    };

    // In the wrapFunc(...) functions below, we assume that the input function always accepts all arguments
    // by non-const reference. This will avoid unnecessary copying when we eventually call the input function.
    // We can't assume the input function accepts arguments by const reference, because the input function
    /// might want to modify the arguments, e.g., when an input function resolves pointers to shared memory
    // for an input SpPackedArray& before forwarding it to an inner function.

    // Typically called from the game thread in EngineService::bindFuncToExecuteOnWorkerThread(...) to get an
    // outer func that executes the given inner func in a try-catch block.

    template <typename TFunc>
    auto wrapFuncToExecuteInTryCatch(const std::string& func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInTryCatch(func_name, func, FuncInfoUtils::getFuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInTryCatch(const std::string& func_name, const TFunc& func, const FuncInfo<TReturn, TArgs...>& fi)
    {
        // The lambda declared below is typically bound to a specific RPC entry point and called from the
        // worker thread by the RPC server. Note that we capture func_name and func by value because we want
        // to guarantee that func_name and func are still accessible after wrapFuncToExecuteInTryCatch(...)
        // returns.

        return [this, func_name, func](TArgs&... args) -> TReturn {

            if (frame_state_ == FrameState::Terminating) {
                return TReturn();
            }

            return executeFuncInTryCatch(func_name, [&func_name, &func, &args...]() -> TReturn {
                if constexpr (std::is_void_v<TReturn>) {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, "...");
                    }

                    func(args...);

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, ".");
                    }

                } else {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, "...");
                    }

                    TReturn return_value = func(args...);

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, ".");
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
        // worker thread by the RPC server. Note that we capture func_name and func by value because we want
        // to guarantee that func_name and func are still accessible after wrapFuncToExecuteInWorkQueueBlocking(...)
        // returns.

        return [this, func_name, func](TArgs&... args) -> TReturn {
            return executeFuncInTryCatch(func_name, [this, &func_name, &func, &args...]() -> TReturn {

                if (request_error_) {
                    SP_LOG("ERROR: In an error state when attempting to execute ", func_name, " returning a default-constructed return value...");
                    return TReturn();
                }

                if (!current_work_queue_) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("ERROR: Current work queue is null when attempting to execute function: ", func_name);
                }
                SP_ASSERT(current_work_queue_);

                if constexpr (std::is_void_v<TReturn>) {
                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, " on work queue ", current_work_queue_->getName(), " (blocking)...");
                    }

                    current_work_queue_->scheduleFunc(func_name, func, args...).get();

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, " on work queue ", current_work_queue_->getName(), " (blocking).");
                    }

                } else {
                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, " on work queue ", current_work_queue_->getName(), " (blocking)...");
                    }

                    TReturn return_value = current_work_queue_->scheduleFunc(func_name, func, args...).get();

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, " on work queue ", current_work_queue_->getName(), " (blocking).");
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
            return executeFuncInTryCatch(func_name, [this, &func_name, &func, &args...]() -> SpFuture {

                if (request_error_) {
                    SP_LOG("ERROR: In an error state when attempting to execute ", func_name, " returning a default-constructed SpFuture...");
                    return SpFuture();
                }

                if (!current_work_queue_) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("ERROR: Current work queue is null when attempting to execute function: ", func_name);
                }
                SP_ASSERT(current_work_queue_);

                if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("Scheduling function ", func_name, " on work queue ", current_work_queue_->getName(), " (non-blocking)...");
                }

                SpFuture future = createFuture(current_work_queue_->scheduleFunc(func_name, func, args...));

                if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("Finished scheduling function ", func_name, " on work queue ", current_work_queue_->getName(), " (non-blocking).");
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
        // to guarantee that func_name and func are still accessible after wrapFuncToExecuteInWorkQueueNonBlockingDiscardReturnValue(...)
        // returns.

        return [this, func_name, func](TArgs&... args) -> void {
            executeFuncInTryCatch(func_name, [this, &func_name, &func, &args...]() -> void {

                if (request_error_) {
                    SP_LOG("ERROR: In an error state when attempting to execute ", func_name, " returning...");
                    return;
                }

                if (!current_work_queue_) {
                    SP_LOG_CURRENT_FUNCTION();
                    SP_LOG("ERROR: Current work queue is null when attempting to execute function: ", func_name);
                }
                SP_ASSERT(current_work_queue_);

                if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("Scheduling function ", func_name, " on work queue ", current_work_queue_->getName(), " (non-blocking, disarding return value)...");
                }

                current_work_queue_->scheduleFunc(func_name, func, args...);

                if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.ENGINE_SERVICE.PRINT_CALL_DEBUG_INFO")) {
                    SP_LOG("Finished scheduling function ", func_name, " on work queue ", current_work_queue_->getName(), " (non-blocking, disarding return value)...");
                }
            });
        };
    }

    // Called from the worker thread and the game thread to execute a function in a try-catch block.

    template <typename TFunc>
    auto executeFuncInTryCatch(const std::string& func_name, const TFunc& func)
    {
        using TReturn = std::invoke_result_t<TFunc>;

        try {
            return func();
        } catch (const std::exception& e) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("ERROR: Caught exception when executing ", func_name, ": ", e.what());
        } catch (...) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("ERROR: Caught unknown exception when executing ", func_name, ".");
        }

        // If we get this far, an exception has occurred, so set the error state, reset the work
        // queue to allow the game thread to proceed, and return a default-constructed object.

        request_error_ = true;

        return TReturn();
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
    static TReturn destroyFuture(SpFuture& future)
    {
        if (future.future_ptr_) {
            SP_ASSERT(future.type_id_ == Std::getTypeIdString<TReturn>());
            std::future<TReturn>* future_ptr = static_cast<std::future<TReturn>*>(future.future_ptr_);
            TReturn result = future_ptr->get();
            delete future_ptr;
            future_ptr = nullptr;
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
            future_ptr = nullptr;
        } else {
            SP_LOG("ERROR: Attempting to destroy a default-constructed SpFuture, returning...");
        }
    }

    std::map<std::string, FuncSignatureRegistry> entry_point_signature_registries_;
    TEntryPointBinder* entry_point_binder_ = nullptr;

    // Work queues. Safe to access on multiple threads.
    std::array<WorkQueue, 4> work_queues_ = {WorkQueue("begin_frame_work_queue_A"), WorkQueue("end_frame_work_queue_A"), WorkQueue("begin_frame_work_queue_B"), WorkQueue("end_frame_work_queue_B")};

    // Only accessed on the worker thread.
    int begin_frame_writable_by_wt_ = 0;
    int begin_frame_being_drained_by_gt_ = 2;

    // Only accessed on the worker thread.
    int end_frame_writable_by_wt_ = 1;
    int end_frame_being_drained_by_gt_ = 3;

    // Only accessed on the worker thread.
    WorkQueue* current_work_queue_ = nullptr;

    // Written and read by the game thread in beginFrame() and endFrame(). Written by the worker thread in engine_service.initialize
    // and engine_service.terminate. Read by the worker thread in the lambda returned by wrapFuncToExecuteInTryCatch(...),
    // which is used when executing all entry points on the worker thread.
    std::atomic<FrameState> frame_state_ = FrameState::Idle;

    // Written by the worker thread in engine_service.initialize and engine_service.terminate to indicate to
    // the game thread that we want to terminate the application. Read by the worker thread in engine_service.begin_frame,
    // engine_service.execute_frame, and engine_service.end_frame. Read by the game thread in beginFrame()
    // and endFrame().
    std::atomic<bool> request_terminate_ = false;

    // Written by the worker thread in engine_service.initialize. Written by the worker thread and the game
    // thread in executeFuncInTryCatch(...) to indicate to the game thread that an error has occurred. Read
    // by the worker thread in engine_service.begin_frame, engine_service.execute_frame, and engine_service.end_frame.
    // Read by the game thread in beginFrame(), endFrame(), and the lambdas returned by the wrapFuncToExecuteInWorkQueue(...)
    // methods.
    std::atomic<bool> request_error_ = false;

    // The mutex is used to coordinate access to the FIFOs, promises, futures, bools defined below.
    std::mutex mutex_;

    // This FIFO is written to by the worker thread in engine_service.begin_frame, engine_service.execute_frame,
    // and in a lambda that is scheduled on the game thread in engine_service.end_frame (if in single-step mode).
    // The FIFO is drained by the game thread in beginFrame() and endFrame().
    boost::circular_buffer<std::pair<int, QueueType>> work_queue_fifo_ = boost::circular_buffer<std::pair<int, QueueType>>(4);

    // These promises and futures are each initialized by the worker thread in engine_service.initialize, engine_service.begin_frame,
    // engine_service.execute_frame, and engine_service.end_frame (if in single-step mode). Subsequently,
    // the promises are only ever set by the game thread in beginFrame(), endFrame(), and terminate(). The
    // futures are only ever read on the worker thread in engine_service.begin_frame, engine_service.execute_frame,
    // and engine_service.end_frame (if in single-step mode). Each bool variable is set by the game thread
    // whenever the corresponding promise is set, and is read by the game thread in shutdown() to avoid
    // deadlocks when shutting down the application.

    std::array<std::promise<void>, 4> work_queue_ready_promises_;
    std::array<std::future<void>, 4> work_queue_ready_futures_;
    std::array<std::atomic<bool>, 4> work_queue_ready_promises_set_ = {false, false, false, false};
};
