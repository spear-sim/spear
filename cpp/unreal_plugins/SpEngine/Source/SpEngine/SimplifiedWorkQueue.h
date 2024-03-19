//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>    // std::same_as
#include <cmath>       // std::nan
#include <functional>
#include <future>
#include <map>
#include <mutex>
#include <string>
#include <utility>     // std::forward, std::move
#include <vector>
#include <type_traits> // std::invoke_result_t

#include <SpCore/Assert.h>
#include <SpCore/BoostAsio.h>


// The purpose of this class is to provide a copy-constructible type that derives from std::packaged_task. This is necessary 
// because boost::asio requires that task objects are copy-constructible, but std::packaged_task is not copy-constructible.
template <typename TFunc>
struct CopyConstructibleTask : std::packaged_task<typename std::invoke_result<TFunc>::type()>
{
    CopyConstructibleTask(TFunc&& func) : std::packaged_task<typename std::invoke_result<TFunc>::type()>(std::forward<TFunc>(func)) {};
};


class WorkQueue {

public:
    WorkQueue() : io_context_(), executor_work_guard_(io_context_.get_executor()) {}

    template <typename TFunc>
    static auto createCopyConstructibleTask(TFunc&& func)
    {
        return CopyConstructibleTask(std::forward<TFunc>(func));
    }

    template <typename TFunc>
    static auto wrapFuncToExecuteInWorkQueueBlocking(WorkQueue& work_queue, TFunc&& func)
    {
        return [&work_queue, func](auto&&... args) -> auto {
            auto task = createCopyConstructibleTask(
                [func, args...]() -> auto {
                    return func(args...);
                });

            auto future = task.get_future();
            boost::asio::post(work_queue.io_context_, std::move(task));
            return future.get();
        };
        //return[func]<typename... TArgs>(TArgs&&... args) -> auto {
        //    return func(std::forward<TArgs>(args)...);
        //};
    }

    void runIOContext()
    {
        // run all scheduled work and wait for executor_work_guard_.reset() to be called from a worker thread
        io_context_.run();

        // reinitialze io_context_ and executor_work_guard_ to prepare for the next call to runSync()
        mutex_.lock();
        io_context_.restart();
        new(&executor_work_guard_) boost::asio::executor_work_guard<boost::asio::io_context::executor_type>(io_context_.get_executor());
        mutex_.unlock();
    }

    void resetWorkGuard()
    {
        // request io_context_.run() to stop executing once all of its scheduled work is finished
        mutex_.lock();
        executor_work_guard_.reset();
        mutex_.unlock();
    }

    boost::asio::io_context io_context_;

private:
    std::mutex mutex_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> executor_work_guard_;
};
