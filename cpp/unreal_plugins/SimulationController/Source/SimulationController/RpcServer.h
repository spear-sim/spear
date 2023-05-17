//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <future>
#include <mutex>
#include <string>
#include <utility>

#include "CoreUtils/Rpclib.h"
#include "SimulationController/BoostAsio.h"

// This class defines an RPC server in which user-specified functions can be bound to run synchronously in a specific
// thread, or asynchronously in a worker thread. Use runAsync() to start the server, and use stopRunAsync() to terminate
// the server. Functions that are bound using bindAsync(...) will run asynchronously in a worker thread, and functions
// that are bound using bindSync(...) will be queued, and will run synchronously when the user calls runSync(). The
// functions that are bound using bindSync() will run in the same thread as the one that calls runSync(). By design,
// runSync() will block indefinitely, even if there is no more synchronous work that has been scheduled. In order to
// stop runSync() from blocking, a user-specified function must call requestStopRunSync() from a worker thread, which
// will make runSync() return as soon as it is finished executing all of its scheduled work. This design gives us precise
// control over where and when user-specified functions are executed within the Unreal Engine game loop.

class RpcServer
{
public:
    template <typename... TArgs>
    explicit RpcServer(TArgs&&... args) : server_(std::forward<TArgs>(args)...), io_context_(), executor_work_guard_(io_context_.get_executor()) {}

    void runAsync(int num_worker_threads)
    {
        server_.async_run(num_worker_threads);
    }

    void stopRunAsync()
    {
        server_.close_sessions();
        server_.stop();
    }

    void runSync()
    {
        // run all scheduled work and wait for executor_work_guard_.reset() to be called from a worker thread
        io_context_.run();

        // reinitialze io_context_ and executor_work_guard_ to prepare for the next call to runSync()
        mutex_.lock();
        io_context_.restart();
        new(&executor_work_guard_) boost::asio::executor_work_guard<boost::asio::io_context::executor_type>(io_context_.get_executor());
        mutex_.unlock();
    }

    void requestStopRunSync()
    {
        // request io_context_.run() to stop executing once all of its scheduled work is finished
        mutex_.lock();
        executor_work_guard_.reset();
        mutex_.unlock();
    }

    template <typename TFunctor>
    void bindAsync(const std::string& name, TFunctor&& functor);

    template <typename TFunctor>
    void bindSync(const std::string& name, TFunctor&& functor);

private:
    rpc::server server_;
    std::mutex mutex_;
    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> executor_work_guard_;
};

namespace detail
{

// The purpose of this MoveWrapper class and moveHandler(...) function are to trick boost::asio into
// accepting move-only handlers. If the handler was actually copied, it would result in a link error.
// See https://stackoverflow.com/a/22891509 for more details.
template <typename TFunctor>
struct MoveWrapper : TFunctor
{
    MoveWrapper(TFunctor&& functor): TFunctor(std::move(functor)) {}

    MoveWrapper(MoveWrapper&&) = default;
    MoveWrapper& operator=(MoveWrapper&&) = default;

    MoveWrapper(const MoveWrapper&);
    MoveWrapper& operator=(const MoveWrapper&);
};

template <typename TFunctor>
auto moveHandler(TFunctor&& functor)
{
    return detail::MoveWrapper<typename std::decay<TFunctor>::type>(std::move(functor));
}

template <typename TClass>
struct FunctionWrapper : FunctionWrapper<decltype(&TClass::operator())> {};

template <typename TClass, typename TReturn, typename... TArgs>
struct FunctionWrapper<TReturn (TClass::*)(TArgs...)> : FunctionWrapper<TReturn (*)(TArgs...)> {};

template <typename TClass, typename TReturn, typename... TArgs>
struct FunctionWrapper<TReturn (TClass::*)(TArgs...) const> : FunctionWrapper<TReturn (*)(TArgs...)> {};

template <class T>
struct FunctionWrapper<T&> : public FunctionWrapper<T> {};

template <class T>
struct FunctionWrapper<T&&> : public FunctionWrapper<T> {};

template <typename TReturn, typename... TArgs>
struct FunctionWrapper<TReturn (*)(TArgs...)>
{
    template <typename TFunctor>
    static auto wrapAsync(TFunctor&& functor)
    {
        return [functor = std::forward<TFunctor>(functor)](TArgs... args) -> TReturn {
            return functor(args...);
        };
    }

    // This function wraps an "inner" functor in an "outer" function with an equivalent signature. The outer
    // function posts the inner functor into a work queue represented by boost::asio::io_context, waits for the
    // inner functor to finish executing, and returns the result. This design enables precise control over
    // where and when user-specified functions are executed. For example, even if the outer function is called
    // from a worker thread, the inner function will be executed on whatever thread flushes the work queue by
    // calling boost::asio::io_context::run().
    //
    // In the RpcServer class above, we use this flexibility to ensure that user-specified functions run at
    // specific times in the Unreal Engine game thread. Whenever we want a previously queued user-specified
    // function to run at a specific time on the game thread, we simply call runSync() from the game thread,
    // which calls boost::asio::io_context::run(). Doing so will run all previously queued user-specified
    // functions that have been bound to the rpc::server using wrapSync(...).

    template <typename TFunctor>
    static auto wrapSync(boost::asio::io_context& io_context, TFunctor&& functor)
    {
        return [&io_context, functor = std::forward<TFunctor>(functor)](TArgs... args) -> TReturn {

            auto task = std::packaged_task<TReturn()>([functor = std::move(functor), args...]() -> TReturn {
                return functor(args...);
            });

            auto future = task.get_future();
            boost::asio::post(io_context, moveHandler(task));

            return future.get();
        };
    }
};

} // namespace detail

template <typename TFunctor>
inline void RpcServer::bindAsync(const std::string& name, TFunctor&& functor)
{
    server_.bind(name, detail::FunctionWrapper<TFunctor>::wrapAsync(std::forward<TFunctor>(functor)));
}

template <typename TFunctor>
inline void RpcServer::bindSync(const std::string& name, TFunctor&& functor)
{
    server_.bind(name, detail::FunctionWrapper<TFunctor>::wrapSync(io_context_, std::forward<TFunctor>(functor)));
}
