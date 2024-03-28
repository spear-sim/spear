//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts> // std::same_as
#include <cmath>    // std::nan
#include <future>
#include <map>
#include <mutex>
#include <string>
#include <utility>  // std::forward, std::move
#include <vector>

#include <SpCore/Assert.h>
#include <SpCore/Boost.h>

template <typename TFunc, typename TReturn, typename... TArgs>
concept CCallable = requires(TFunc func, TArgs... args) {
    { func(args...) } -> std::same_as<TReturn>;
};

template <typename TClass>
struct FuncInfo : public FuncInfo<decltype(&TClass::operator())> {};

template <typename TClass, typename TReturn, typename... TArgs>
struct FuncInfo<TReturn(TClass::*)(TArgs...)> : public FuncInfo<TReturn(*)(TArgs...)> {};

template <typename TClass, typename TReturn, typename... TArgs>
struct FuncInfo<TReturn(TClass::*)(TArgs...) const> : public FuncInfo<TReturn(*)(TArgs...)> {};

template <typename TReturn, typename... TArgs>
struct FuncInfo<TReturn(*)(TArgs...)> {};


// The purpose of this class is to provide a copy-constructible type that derives from std::packaged_task. This is necessary 
// because boost::asio requires that task objects are copy-constructible, but std::packaged_task is not copy-constructible.
template <typename TReturn>
struct CopyConstructibleTask : std::packaged_task<TReturn()>
{
    CopyConstructibleTask(auto&& func) : std::packaged_task<TReturn()>(std::forward<decltype(func)>(func)) {};
};


class WorkQueue {

public:
    WorkQueue() : io_context_(), executor_work_guard_(io_context_.get_executor()) {}

    template <typename TFunc, typename TReturn, typename... TArgs> requires CCallable<TFunc, TReturn, TArgs...>
    static CopyConstructibleTask<TReturn> createCopyConstructibleTaskImpl(TFunc&& func, const FuncInfo<TReturn(*)(TArgs...)>& fi)
    {
        return CopyConstructibleTask<TReturn>(std::forward<TFunc>(func));
    }

    template <typename TFunc>
    static auto createCopyConstructibleTask(TFunc&& func)
    {
        return createCopyConstructibleTaskImpl(std::forward<TFunc>(func), FuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs> requires CCallable<TFunc, TReturn, TArgs...>
    TReturn scheduleAndExecuteTaskBlockingImpl(const TFunc& func, const FuncInfo<TReturn(*)(TArgs&...)>& fi, auto&... args)
    {
        // Note that we capture func and args by value because we want to guarantee that they are both
        // still accessible after this scheduleAndExecuteTaskBlockingImpl(...) function returns. Strictly
        // speaking, this guarantee is not necessary for this blocking implementation, but we capture
        // by value anyway for consistency with our non-blocking implementation.
        auto task = createCopyConstructibleTask(
            [func, args...]() -> TReturn {
                return func(args...);
            });

        std::future<TReturn> future = task.get_future();
        boost::asio::post(io_context_, std::move(task));
        return future.get();
    }

    template <typename TFunc>
    auto scheduleAndExecuteTaskBlocking(const TFunc& func, auto&... args)
    {
        return scheduleAndExecuteTaskBlockingImpl(func, FuncInfo<TFunc>(), args...);
    }

    template <typename TFunc, typename TReturn, typename... TArgs> requires CCallable<TFunc, TReturn, TArgs...>
    static auto wrapFuncToExecuteInWorkQueueBlockingImpl(WorkQueue& work_queue, TFunc&& func, const FuncInfo<TReturn(*)(TArgs&...)>& fi)
    {
        // Note that we capture func by value because we want to guarantee that func is still
        // accessible after this wrapFuncToExecuteInWorkQueueBlocking(...) function returns.

        // Note also that we assume that the user's function accepts arguments by const reference.
        // This will avoid an unnecessary copy when the surrounding code (e.g., the RPC server)
        // calls the function returned here. Deeper inside our implementation, we will eventually
        // copy these arguments so they are guaranteed to be accessible when we eventually execute
        // the user's function, but we want to avoid copying wherever possible.
        return [&work_queue, func](TArgs&... args) -> TReturn {
            return work_queue.scheduleAndExecuteTaskBlocking(func, args...);
        };
    }

    template <typename TFunc>
    static auto wrapFuncToExecuteInWorkQueueBlocking(WorkQueue& work_queue, TFunc&& func)
    {
        return wrapFuncToExecuteInWorkQueueBlockingImpl(work_queue, std::forward<TFunc>(func), FuncInfo<TFunc>());
    }

    void run()
    {
        // run all scheduled work and wait for executor_work_guard_.reset() to be called from a worker thread
        io_context_.run();

        // reinitialze io_context_ and executor_work_guard_ to prepare for the next call to runSync()
        mutex_.lock();
        io_context_.restart();
        new(&executor_work_guard_) boost::asio::executor_work_guard<boost::asio::io_context::executor_type>(io_context_.get_executor());
        mutex_.unlock();
    }

    void reset()
    {
        // request io_context_.run() to stop executing once all of its scheduled work is finished
        mutex_.lock();
        executor_work_guard_.reset();
        mutex_.unlock();
    }

private:
    std::mutex mutex_;
    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> executor_work_guard_;
};
