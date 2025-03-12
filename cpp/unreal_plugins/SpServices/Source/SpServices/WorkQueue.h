//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <atomic>
#include <exception>
#include <future>
#include <map>
#include <mutex>       // std::lock_guard
#include <string>
#include <utility>     // std::forward, std::move
#include <vector>
#include <type_traits> // std::invoke_result_t

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"

#include "SpServices/FuncInfo.h"

enum class WorkQueueErrorState
{
    NoError = 0,
    Error   = 1
};

class WorkQueue {

public:
    WorkQueue() : io_context_(), executor_work_guard_(io_context_.get_executor()) {}

    // called from the game thread in EngineService::PIEStartedHandler() to reset the work queue's state,
    // which is useful if a previous PIE session run put the work queue in an error state.
    void initialize();

    // typically called from the game thread in EngineService::beginFrameHandler(...) and EngineService::endFrameHandler(...)
    // to block the game thread indefinitely, while the WorkQueue waits for and executes incoming work
    void run();

    // typically called from an RPC worker thread in the "engine_service.execute_frame" and "engine_service.end_frame"
    // entry points to instruct the WorkQueue that it can stop blocking, as soon as it is finished executing
    // all of its scheduled work
    void reset();

    // typically called from the game thread in EngineService::bindFuncToExecuteOnGameThread(...) to get an
    // outer func that executes a given inner func via a given WorkQueue
    template <typename TFunc>
    static auto wrapFuncToExecuteInWorkQueueBlocking(WorkQueue& work_queue, const std::string& func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInWorkQueueBlockingImpl(work_queue, func_name, func, FuncInfo<TFunc>());
    }

private:
    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    static auto wrapFuncToExecuteInWorkQueueBlockingImpl(WorkQueue& work_queue, const std::string& func_name, const TFunc& func, const FuncInfo<TReturn(*)(TArgs...)>& fi)
    {
        // The lambda returned here is typically bound to a specific RPC entry point and called from a worker
        // thread by the RPC server.

        // Note that we capture func_name and func by value because we want to guarantee that func is still
        // accessible after this wrapFuncToExecuteInWorkQueueBlocking(...) function returns.

        // Note also that we assume that the user's function always accepts all arguments by non-const
        // reference. This will avoid unnecessary copying when we eventually call the user's function. We
        // can't assume the user's function accepts arguments by const reference, because the user's function
        // might want to modify the arguments, e.g., when a user function resolves pointers to shared memory
        // for an input SpPackedArray& before forwarding it to an inner function.

        return [&work_queue, func_name, func](TArgs&... args) -> TReturn {
            return work_queue.scheduleAndExecuteFuncBlocking(func_name, func, args...);
        };
    }

    template <typename TFunc, typename... TArgs> requires
        CFuncIsCallableWithArgs<TFunc, TArgs&...>
    auto scheduleAndExecuteFuncBlocking(const std::string& func_name, const TFunc& func, TArgs&... args)
    {
        using TReturn = std::invoke_result_t<TFunc, TArgs&...>;

        if (error_state_ == WorkQueueErrorState::Error) {
            SP_LOG("Work queue is in an error state, returning immediately from func: ", func_name);
            return TReturn();
        }

        // The lambda declared below is typically executed on the game thread when run() is called, i.e.,
        // during EngineService::beginFrameHandler(...) or EngineService::endFrameHandler(...)

        // Note that we capture func and args... by value because we want to guarantee that they are both
        // still accessible after scheduleAndExecuteTaskBlocking(...) returns. Strictly speaking, this
        // guarantee is not necessary for this blocking implementation, but we capture by value anyway for
        // consistency with a non-blocking implementation.

        // Even in a non-blocking implementation, we could technically capture func by reference. This is
        // because, in practice, the lifetime of func corresponds to the lifetime of the lambda declared in
        // wrapFuncToExecuteInWorkQueueBlockingImpl(...) above, which in turn corresponds to the the lifetime
        // of the RPC server. Moreover, the lambda declared below only ever executes when run() is called,
        // i.e., during EngineService::beginFrameHandler(...) or EngineService::endFrameHandler(...), and
        // the RPC server (and therefore func) is guaranteed to be accessible inside these EngineService
        // functions. So, even if we capture func by reference, it is guaranteed to be accessible whenever
        // the lambda below is executed. However, the WorkQueue class should not depend on this high-level
        // system behavior, so we insist on capturing func by value, even in a non-blocking implementation.

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
            [func, args...]() mutable -> TReturn {
                return func(args...);
            });

        std::future<TReturn> future = task.get_future(); // need to call get_future() before calling std::move(...)
        boost::asio::post(io_context_, std::move(task));

        // Wait until the game thread is finished executing the lambda and return the result.
        try {
            return future.get();

        // If we get this far, an exception has occurred, so we store the exception for later, set the error state,
        // allow the game thread to proceed by calling reset(), and return a default-constructed value.
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lock(mutex_);
            SP_LOG("Caught exception when executing ", func_name, ": ", e.what());
            error_state_ = WorkQueueErrorState::Error;
            exception_ptr_ = std::current_exception();
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            SP_LOG("Caught unknown exception when executing ", func_name, ".");
            error_state_ = WorkQueueErrorState::Error;
            exception_ptr_ = std::current_exception();
        }

        reset();
        return TReturn();
    }

    // The purpose of this class is to provide a copy-constructible type that derives from std::packaged_task.
    // This is necessary because boost::asio requires that task objects are copy-constructible, but std::packaged_task
    // is not copy-constructible.

    // We use auto&& because we want to preserve and forward the const-ness and rvalue-ness of func.
    template <typename TReturn>
    struct CopyConstructiblePackagedTask : std::packaged_task<TReturn()>
    {
        CopyConstructiblePackagedTask(auto&& func) : std::packaged_task<TReturn()>(std::forward<decltype(func)>(func)) {};
    };

    std::mutex mutex_;

    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> executor_work_guard_;

    WorkQueueErrorState error_state_ = WorkQueueErrorState::NoError;
    std::exception_ptr exception_ptr_ = nullptr;
};
