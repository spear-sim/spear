#pragma once

// Modified from CARLA (https://carla.org)

#include <future>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>

#include "Asio.h"
#include "Rpclib.h"

// This file defines an RPC server in which functions can be bound to run synchronously or
// asynchronously. Use launchWorkerThreads() to start the worker threads, and use stop() to
// terminate the server. Functions that are bound using bindAsync() will run asynchronously
// in the worker threads, and functions that are bound using bindSync() will run synchronously
// in the game thread when the game thread calls runSync(). By design, runSync() will block
// indefinitely, even if there is no more synchronous work that has been scheduled. Call
// unblockRunSyncWhenFinishedExecuting() from another thread to make runSync() return as soon
// as it is finished executing all scheduled work. This design gives us precise control over
// where and when function calls are executed within the Unreal Engine game loop.

namespace detail
{
template <typename FunctorT>
struct MoveWrapper : FunctorT
{
    MoveWrapper(FunctorT&& f): FunctorT(std::move(f)) {}

    MoveWrapper(MoveWrapper&&) = default;
    MoveWrapper& operator=(MoveWrapper&&) = default;

    MoveWrapper(const MoveWrapper&);
    MoveWrapper& operator=(const MoveWrapper&);
};

} // namespace detail

// hack to trick asio into accepting move-only handlers, if the handler were actually copied it would result in a link error. @see https://stackoverflow.com/a/22891509.
template <typename FunctorT>
auto moveHandler(FunctorT&& func)
{
    using F = typename std::decay<FunctorT>::type;
    return detail::MoveWrapper<F>{std::move(func)};
}


class RpcServer
{
public:
    template <typename... Args>
    explicit RpcServer(Args&&... args);

    template <typename FunctorT>
    void bindSync(const std::string& name, FunctorT&& functor);

    template <typename FunctorT>
    void bindAsync(const std::string& name, FunctorT&& functor);

    void launchWorkerThreads(size_t worker_threads)
    {
        server_.async_run(worker_threads);
    }

    void runSync()
    {
        io_context_.run();

        // reinitialze the io_context and work guard after runSync to prepare for next run
        mutex_.lock();
        io_context_.restart();
        new(&executor_work_guard_) asio::executor_work_guard<asio::io_context::executor_type>(io_context_.get_executor());
        mutex_.unlock();
    }

    void unblockRunSyncWhenFinishedExecuting()
    {
        // allow io_context to return from run()
        mutex_.lock();
        executor_work_guard_.reset();
        mutex_.unlock();
    }

    // warning does not stop the game thread.
    void stop()
    {
        server_.close_sessions();
        server_.stop();
    }

private:
    ::rpc::server server_;
    std::mutex mutex_;
    asio::io_context io_context_;
    asio::executor_work_guard<asio::io_context::executor_type> executor_work_guard_;
};

namespace detail
{
template <typename T>
struct FunctionWrapper : FunctionWrapper<decltype(&T::operator())> {};

template <typename C, typename R, typename... Args>
struct FunctionWrapper<R (C::*)(Args...)> : FunctionWrapper<R (*)(Args...)> {};

template <typename C, typename R, typename... Args>
struct FunctionWrapper<R (C::*)(Args...) const> : FunctionWrapper<R (*)(Args...)> {};

template <class T>
struct FunctionWrapper<T&> : public FunctionWrapper<T> {};

template <class T>
struct FunctionWrapper<T&&> : public FunctionWrapper<T> {};

template <typename R, typename... Args>
struct FunctionWrapper<R (*)(Args...)>
{
    // Wraps @a functor into a function type with equivalent signature. The wrap function returned. When called, posts @a functor into the io_context;
    // if the client called this method synchronously, waits for the posted task to finish, otherwise returns immediately.
    // This way, no matter from which thread the wrap function is called, the @a functor provided is always called from the context of the io_context.
    // I.e., we can use the io_context to run tasks on a specific thread (e.g. game thread).
    template <typename FuncT>
    static auto wrapSyncCall(asio::io_context& io_context, FuncT&& functor)
    {
        return
            [&io_context, functor = std::forward<FuncT>(functor)](Args... args) -> R {
                auto task = std::packaged_task<R()>(
                    [functor = std::move(functor), args...]() {
                        return functor(args...);
                    });
                // Post task and wait for result.
                auto result = task.get_future();
                asio::post(io_context, moveHandler(task));
                return result.get();
            };
    }

    template <typename FuncT>
    static auto wrapAsyncCall(FuncT&& functor)
    {
        return [functor = std::forward<FuncT>(functor)](Args... args) -> R {
            {
                return functor(args...);
            }
        };
    }
};
} // namespace detail

template <typename... Args>
inline RpcServer::RpcServer(Args&&... args) : server_(std::forward<Args>(args)...), io_context_(), executor_work_guard_(io_context_.get_executor())
{
    // Throwing an exception will cause the server to write an error response. This call will make it also suppress the exception (note that this is not default because this behavior might hide errors in the code)
    // @Todo: Don't think this should be suppressed!!
    // server_.suppress_exceptions(true);
}

template <typename FunctorT>
inline void RpcServer::bindSync(const std::string& name, FunctorT&& functor)
{
    using Wrapper = detail::FunctionWrapper<FunctorT>;
    server_.bind(name, Wrapper::wrapSyncCall(io_context_, std::forward<FunctorT>(functor)));
}

template <typename FunctorT>
inline void RpcServer::bindAsync(const std::string& name, FunctorT&& functor)
{
    using Wrapper = detail::FunctionWrapper<FunctorT>;
    server_.bind(name, Wrapper::wrapAsyncCall(std::forward<FunctorT>(functor)));
}
