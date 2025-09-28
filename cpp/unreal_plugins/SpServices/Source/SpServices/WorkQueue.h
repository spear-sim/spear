//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <future>
#include <memory>      // std::make_unique, std::unique_ptr
#include <mutex>       // std::lock_guard
#include <string>
#include <utility>     // std::forward, std::move
#include <type_traits> // std::invoke_result_t

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Config.h"

class SPSERVICES_API WorkQueue
{
public:
    WorkQueue() = delete;
    WorkQueue(const std::string& name) { name_ = name; }

    // Typically called from the worker thread from the engine_service.initialize entry point to initialize
    // (or re-initialize) the work queue.

    void initialize()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        io_context_ = std::make_unique<boost::asio::io_context>();
        SP_ASSERT(io_context_);
        executor_work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(io_context_->get_executor());
        SP_ASSERT(executor_work_guard_);
    }

    // Typically called from the worker thread from the engine_service.terminate entry point to shut down
    // the work queue.

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
        io_context_->run();

        {
            std::lock_guard<std::mutex> lock(mutex_);

            io_context_->restart();
            executor_work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(io_context_->get_executor());
        }
    }

    // Typically called from the engine_service.execute_frame and engine_service.end_frame entry points to
    // instruct the WorkQueue that it can stop blocking, as soon as it is finished executing all of its
    // scheduled work.

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        SP_ASSERT(executor_work_guard_);

        // request io_context_->run() to stop executing once all of its scheduled work is finished
        executor_work_guard_->reset();
    }

    // Typically called from a lambda that is bound to a specific RPC entry point and is called from the worker thread.

    template <typename TFunc, typename... TArgs> requires
        CFuncIsCallableWithArgs<TFunc, TArgs&...>
    auto scheduleFunc(const std::string& func_name, const TFunc& func, TArgs&... args)
    {
        using TReturn = std::invoke_result_t<TFunc, TArgs&...>;

        SP_ASSERT(io_context_);

        // This function is typically called from the worker thread, but the lambda declared below is
        // typically executed on the game thread when run() is called, i.e., during EngineService::beginFrameHandler(...)
        // or EngineService::endFrameHandler(...)

        // Note that we capture func and args... by value because we want to guarantee that they are both
        // still accessible after scheduleAndExecuteTaskBlocking(...) returns.

        // Even in a non-blocking implementation, we could technically capture func by reference. This is
        // because, in practice, the lifetime of func corresponds to the lifetime of the lambda declared in
        // EngineService::wrapFuncToExecuteInWorkQueueBlocking(...) and other similar functions, which in
        // turn corresponds to the the lifetime of the RPC server. Moreover, the lambda declared below only
        // ever executes when run() is called, i.e., during EngineService::beginFrame(...) or EngineService::endFrame(...),
        // and the RPC server (and therefore func) is guaranteed to be accessible inside these EngineService
        // functions. So, even if we capture func by reference, it is guaranteed to be accessible whenever
        // the lambda below is executed. However, this low-level WorkQueue class should not depend on this
        // high-level system behavior, so we insist on capturing func by value, even in a non-blocking
        // implementation.

        // Since we capture args... by value, it is deep-copied into the lambda object constructed below. But
        // the user's function accepts all arguments by non-const reference, so args... is not copied again
        // when calling the user's function from inside the lambda body.

        // The mutable keyword is required because otherwise args... will be treated as a const member
        // variable inside the lambda body. This is because, by default, capturing variables by value is
        // equivalent to declaring them as const member variables in the anonymous lambda class. It is
        // impossible to pass any const member variable to any function by non-const reference, but we need
        // to pass args... by non-const reference to the user's function. So we use the mutable keyword to
        // force args... to be treated as a non-const member variable inside the lambda body.

        auto task = CopyConstructiblePackagedTask<TReturn>(
            [this, func_name, func, args...]() mutable -> TReturn {

                // If TReturn is void, we cannot declare a variable of type TReturn, so we need some form of
                // template specialization. Since templated functions cannot be partially specialized, we use
                // if constexpr to achieve our desired specialization.

                if constexpr (std::is_void_v<TReturn>) {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.WORK_QUEUE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, " on work queue ", getName(), "...");
                    }

                    func(args...);

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.WORK_QUEUE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, " on work queue ", getName(), ".");
                    }

                } else {

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.WORK_QUEUE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Executing function ", func_name, " on work queue ", getName(), "...");
                    }

                    TReturn return_value = func(args...);

                    if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.WORK_QUEUE.PRINT_CALL_DEBUG_INFO")) {
                        SP_LOG("Finished executing function ", func_name, " on work queue ", getName(), ".");
                    }

                    return return_value;
                }
            });

        std::future<TReturn> future = task.get_future(); // need to call get_future() before calling std::move(...)
        boost::asio::post(*io_context_, std::move(task));

        return future;
    }

    std::string getName() { return name_; }

private:

    // The purpose of this class is to provide a copy-constructible type that derives from std::packaged_task.
    // This is necessary because boost::asio requires that task objects are copy-constructible, but std::packaged_task
    // is not copy-constructible.

    // We use auto&& because we want to preserve and forward the const-ness and rvalue-ness of func.
    template <typename TReturn>
    struct CopyConstructiblePackagedTask : std::packaged_task<TReturn()>
    {
        CopyConstructiblePackagedTask(auto&& func) : std::packaged_task<TReturn()>(std::forward<decltype(func)>(func)) {};
    };

    std::string name_;

    std::mutex mutex_;
    std::unique_ptr<boost::asio::io_context> io_context_ = nullptr;
    std::unique_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> executor_work_guard_ = nullptr;
};
