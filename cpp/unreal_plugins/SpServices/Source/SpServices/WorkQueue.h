//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint64_t

#include <atomic>
#include <future>      // std::packaged_task
#include <memory>      // std::make_shared, std::make_unique, std::unique_ptr
#include <mutex>       // std::lock_guard
#include <string>
#include <type_traits> // std::invoke_result_t
#include <utility>     // std::forward, std::move

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Config.h"

#include "SpServices/FuncInfo.h"

class WorkQueue
{
public:
    WorkQueue() = delete;
    WorkQueue(const std::string& name) { name_ = name; }

    // Typically called from the worker thread from the engine_service.initialize entry point to initialize
    // (or re-initialize) the work queue.

    void initialize()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Reset these unique pointers like we do in terminate() in case the work queue wasn't shut down correctly,
        // e.g., if there was an error in the user's Python code.
        executor_work_guard_ = nullptr;
        io_context_ = nullptr;

        io_context_ = std::make_unique<boost::asio::io_context>();
        SP_ASSERT(io_context_);
        executor_work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(io_context_->get_executor());
        SP_ASSERT(executor_work_guard_);
    }

    // Typically called from the worker thread from the engine_service.terminate entry point, or from the game
    // thread in EngineService::shutdown(), to shut down the work queue. Destroying io_context_ destroys any
    // tasks that are still pending without executing them, so any future waiting on one of these tasks will
    // throw a broken-promise exception rather than waiting forever. Must not be called while another thread
    // is inside run() or pollOne().

    void terminate()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        executor_work_guard_ = nullptr;
        io_context_ = nullptr;
    }

    // Typically called from the game thread in EngineService::beginFrame(...) and EngineService::endFrame(...)
    // to block the game thread indefinitely, while the WorkQueue waits for and executes incoming work.

    void run()
    {
        SP_ASSERT(io_context_);

        // run all scheduled work and wait for executor_work_guard_->reset() to be called
        beginRun();
        io_context_->run();
        endRun();

        {
            std::lock_guard<std::mutex> lock(mutex_);

            io_context_->restart();
            executor_work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(io_context_->get_executor());
            SP_ASSERT(executor_work_guard_);
        }
    }

    std::size_t pollOne()
    {
        SP_ASSERT(io_context_);

        return io_context_->poll_one();
    }

    // Typically called from a lambda that is bound to a specific RPC entry point and is called from the worker thread.

    template <typename TFunc, typename... TArgs> requires
        CFuncIsCallableWithArgs<TFunc, TArgs&...>
    auto scheduleFunc(const std::string& func_name, const TFunc& func, TArgs&... args)
    {
        using TReturn = std::invoke_result_t<TFunc, TArgs&...>;

        // This function is typically called from the worker thread, but the lambda declared below is
        // typically executed on the game thread when run() is called, i.e., during EngineService::beginFrameHandler(...)
        // or EngineService::endFrameHandler(...)

        // Note that we capture func and args... by value because we want to guarantee that they are both
        // still accessible after scheduleFunc(...) returns. We would not need to do this if scheduleFunc(...)
        // also executed func synchronously inside this function, but since the lambda below will be executed
        // asynchronously, we need to make sure that func and args... will still be available when it is
        // finally executed.

        // Even though the lambda below executes asynchronously, we could technically capture func by
        // reference due to some implementation details in the rest of our system. In particular, the
        // lifetime of func corresponds to the lifetime of the lambdas declared in
        // EngineService::wrapFuncToExecuteInWorkQueueBlocking(...) and other similar functions, which in
        // turn corresponds to the lifetime of the RPC server. Moreover, the lambda declared below can only
        // ever execute when run() is called, i.e., during EngineService::beginFrame(...) or
        // EngineService::endFrame(...), and the RPC server (and therefore func) is guaranteed to be
        // accessible inside these EngineService functions. So, even if we captured func by reference, it is
        // guaranteed to be accessible whenever the lambda below is executed. However, this low-level
        // WorkQueue class should not depend on this high-level system behavior, so we insist on capturing
        // func by value.

        // Since we capture args... by value, it is deep-copied into the lambda object constructed below. But
        // the user's function accepts all arguments by non-const reference, so args... is not copied again
        // when calling the user's function from inside the lambda body.

        // The mutable keyword is required below because otherwise args... will be treated as a const member
        // variable inside the lambda body. This is because, by default, capturing variables by value is
        // equivalent to declaring them as const member variables in the anonymous lambda class. It is
        // impossible to pass any const member variable to any function by non-const reference, but we need
        // to pass args... by non-const reference to the user's function, e.g., so the user's function can
        // resolve references to shared memory, which involves modifying function arguments in-place. So we
        // use the mutable keyword to force args... to be treated as a non-const member variable in the
        // anonymous lambda class for the lambda declared below, and therefore as a non-const variable in the
        // lambda body itself.

        std::shared_ptr<std::packaged_task<TReturn()>> inner_task_ptr = std::make_shared<std::packaged_task<TReturn()>>(
            [this, func_name, func, args...]() mutable -> TReturn {

                // If TReturn is void, we cannot declare a variable of type TReturn, so we need some form of
                // template specialization. Since templated functions cannot be partially specialized, we use
                // if constexpr to achieve our desired specialization.

                // We call endExecuteFunc() whether func returns or throws, and we rethrow any exception thrown
                // by func, so std::packaged_task can store it in the future.

                if constexpr (std::is_void_v<TReturn>) {

                    if (Config::get<bool>("SP_SERVICES.WORK_QUEUE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, " on work queue ", getName(), "...");
                    }

                    beginExecuteFunc();
                    try {
                        func(args...);
                        endExecuteFunc();
                    } catch (...) {
                        endExecuteFunc();
                        throw;
                    }

                    if (Config::get<bool>("SP_SERVICES.WORK_QUEUE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, " on work queue ", getName(), ".");
                    }

                } else {

                    if (Config::get<bool>("SP_SERVICES.WORK_QUEUE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, " on work queue ", getName(), "...");
                    }

                    beginExecuteFunc();
                    try {
                        TReturn return_value = func(args...);
                        endExecuteFunc();

                        if (Config::get<bool>("SP_SERVICES.WORK_QUEUE.PRINT_CALL_DEBUG_INFO")) {
                            SP_LOG("Finished executing function ", func_name, " on work queue ", getName(), ".");
                        }

                        return return_value;
                    } catch (...) {
                        endExecuteFunc();
                        throw;
                    }
                }
            });

        std::future<TReturn> future = inner_task_ptr->get_future();

        // Note that boost::asio::post(...) requires submitted tasks to be copy-constructible, but
        // std::packaged_task objects are not copy-constructible, so we need to submit our task via the
        // following copy-constructible wrapper.

        auto outer_task = [inner_task_ptr]() -> void {
            (*inner_task_ptr)();
        };

        // We hold mutex_ while posting, so terminate() can't destroy io_context_ while we're using it. If the
        // work queue has been terminated, we discard the task without executing it, just like terminate()
        // discards any tasks that are still pending, so the caller's future will throw a broken-promise
        // exception rather than waiting forever.

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (io_context_) {
                boost::asio::post(*io_context_, std::move(outer_task));
            } else {
                SP_LOG("ERROR: Work queue ", getName(), " has been terminated, discarding function ", func_name, "...");
            }
        }

        return future;
    }

    // Typically called from the engine_service.execute_frame and engine_service.end_frame entry points to
    // instruct the WorkQueue that it can stop blocking, as soon as it is finished executing all of the work that
    // was scheduled before calling scheduleReset(...). We schedule the reset as work rather than resetting
    // immediately, because the Python code can be ahead of the game thread, so this function might be called
    // before the game thread calls run() to execute the work that was scheduled before calling this function, so
    // scheduleReset() guarantees that the reset() operation happens after the call to run().

    void scheduleReset(const std::string& func_name)
    {
        scheduleFunc(func_name, [this]() -> void {
            reset();
        });
    }

    // Returns a nonzero ID that is unique to the game thread's current idle period, i.e., while it is in run()
    // waiting for work, or 0 if the game thread isn't in run() or is executing a work item. Can be called from any
    // thread, e.g., to determine if the game thread has been waiting in run() for work that will never arrive.
    uint64_t getIdleId() const { return idle_id_; }

    std::string getName() const { return name_; }

private:
    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        SP_ASSERT(executor_work_guard_);

        // request io_context_->run() to stop executing once all of its scheduled work is finished
        executor_work_guard_->reset();
    }

    // The functions below are called on the game thread to maintain idle_id_. The game thread is considered idle
    // while it is in run() and isn't executing a work item. Whenever the game thread becomes idle, we assign a new
    // idle ID, and whenever it stops being idle, we set idle_id_ to 0. Work items can also be executed outside of
    // run() by pollOne(), e.g., when the editor work queue is drained during the editor tick. In this case, in_run_
    // is false in beginExecuteFunc() and endExecuteFunc(), idle_id_ is already 0, and we leave it unchanged so the
    // game thread isn't considered idle after the work item finishes executing.

    void beginRun()
    {
        in_run_ = true;
        beginIdleIdActive();
    }

    void endRun()
    {
        endIdleIdActive();
        in_run_ = false;
    }

    void beginExecuteFunc()
    {
        if (in_run_) {
            endIdleIdActive();
        }
    }

    void endExecuteFunc()
    {
        if (in_run_) {
            beginIdleIdActive();
        }
    }

    void beginIdleIdActive()
    {
        // 0 is reserved to mean "not idle"
        idle_counter_++;
        if (idle_counter_ == 0) {
            idle_counter_++;
        }
        idle_id_ = idle_counter_;
    }

    void endIdleIdActive()
    {
        idle_id_ = 0;
    }

    std::string name_;
    bool in_run_ = false;               // only accessed from the game thread
    uint64_t idle_counter_ = 0;         // only accessed from the game thread
    std::atomic<uint64_t> idle_id_ = 0; // accessed from any thread

    std::mutex mutex_;
    std::unique_ptr<boost::asio::io_context> io_context_;
    std::unique_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> executor_work_guard_;
};
