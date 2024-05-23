//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>    // std::same_as
#include <future>
#include <map>
#include <mutex>
#include <string>
#include <utility>     // std::forward, std::move
#include <vector>
#include <type_traits> // std::invoke_result_t

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"

template <typename TFunc>
concept CFuncIsCallableWithNoArgs = std::is_invocable_v<TFunc>;

template <typename TFunc, typename... TArgs>
concept CFuncIsCallableWithArgs = std::is_invocable_v<TFunc, TArgs...>;

template <typename TFunc, typename TReturn, typename... TArgs>
concept CFuncReturnsAndIsCallableWithArgs = std::same_as<TReturn, std::invoke_result_t<TFunc, TArgs...>>;

class WorkQueue {

public:
    WorkQueue() : io_context_(), executor_work_guard_(io_context_.get_executor()) {}

    // typically called from the game thread in EngineService::beginFrameHandler(...) and EngineService::endFrameHandler(...)
    void run();

    // typically called from a worker thread in the "engine_service.tick" and "engine_service.end_tick" entry points
    void reset();

    // typically called from the game thread in EngineService::bindFuncUnreal(...)
    template <typename TFunc>
    static auto wrapFuncToExecuteInWorkQueueBlocking(WorkQueue& work_queue, TFunc&& func)
    {
        return wrapFuncToExecuteInWorkQueueBlockingImpl(work_queue, std::forward<TFunc>(func), FuncInfo<TFunc>());
    }

private:
    template <typename TClass>
    struct FuncInfo : public FuncInfo<decltype(&TClass::operator())> {};

    template <typename TClass, typename TReturn, typename... TArgs>
    struct FuncInfo<TReturn(TClass::*)(TArgs...)> : public FuncInfo<TReturn(*)(TArgs...)> {};

    template <typename TClass, typename TReturn, typename... TArgs>
    struct FuncInfo<TReturn(TClass::*)(TArgs...) const> : public FuncInfo<TReturn(*)(TArgs...)> {};

    template <class T>
    struct FuncInfo<T&> : public FuncInfo<T> {};

    template <typename TReturn, typename... TArgs>
    struct FuncInfo<TReturn(*)(TArgs...)> {};

    template <typename TFunc, typename TReturn, typename... TArgs> requires CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs...>
    static auto wrapFuncToExecuteInWorkQueueBlockingImpl(WorkQueue& work_queue, TFunc&& func, const FuncInfo<TReturn(*)(TArgs&...)>& fi)
    {
        // Note that we capture func by value because we want to guarantee that func is still accessible
        // after this wrapFuncToExecuteInWorkQueueBlocking(...) function returns.

        // Note also that we assume that the user's function accepts arguments by const reference. This
        // will avoid an unnecessary copy when the surrounding code (e.g., the RPC server) calls the
        // function returned here. Deeper inside our implementation, we will eventually copy these
        // arguments so they are guaranteed to be accessible when we eventually execute the user's
        // function, but we want to avoid copying wherever possible.

        // The lambda returned here is typically bound to a specific RPC entry point and called from a worker thread.
        return [&work_queue, func](TArgs&... args) -> TReturn {
            return work_queue.scheduleAndExecuteTaskBlocking(func, args...);
        };
    }

    template <typename TFunc, typename... TArgs> requires CFuncIsCallableWithArgs<TFunc, TArgs...>
    auto scheduleAndExecuteTaskBlocking(const TFunc& func, TArgs... args)
    {
        using TReturn = std::invoke_result_t<TFunc, TArgs...>;

        // Note that we capture func and args by value because we want to guarantee that they are both
        // still accessible after scheduleAndExecuteTaskBlocking(...) returns. Strictly speaking, this
        // guarantee is not necessary for this blocking implementation, but we capture by value anyway
        // for consistency with our non-blocking implementation.
        auto task = CopyConstructiblePackagedTask<TReturn>(
            [func, args...]() -> TReturn {
                return func(args...);
            });

        std::future<TReturn> future = task.get_future(); // need to call get_future() before calling std::move(...)
        boost::asio::post(io_context_, std::move(task));
        return future.get();
    }

    // The purpose of this class is to provide a copy-constructible type that derives from std::packaged_task. This is necessary 
    // because boost::asio requires that task objects are copy-constructible, but std::packaged_task is not copy-constructible.
    template <typename TReturn>
    struct CopyConstructiblePackagedTask : std::packaged_task<TReturn()>
    {
        CopyConstructiblePackagedTask(auto&& func) : std::packaged_task<TReturn()>(std::forward<decltype(func)>(func)) {};
    };

    boost::asio::io_context io_context_;
    std::mutex mutex_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> executor_work_guard_;
};
