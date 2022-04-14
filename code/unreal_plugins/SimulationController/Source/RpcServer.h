// Modified from Carla (https://carla.org) project, thanks to MIT license.

#pragma once

#include <future>
#include <string>
#include <utility>

#include "Asio.h"
#include "MoveHandler.h"
#include "Rpclib.h"

// An RPC server in which functions can be bind to run synchronously or asynchronously.
// Use `launchWorkerThreads` to start the worker threads, and use `runSync`, to service work on the Game thread. 
// Functions that are bind using `bindAsync` will run asynchronously in the worker threads.
// Functions that are bind using `bindSync` will run within `runSync` function.

class RpcServer
{
public:
    using work_guard_type = asio::executor_work_guard<asio::io_context::executor_type>;

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
        io_context_.restart();
        new(&work_guard_) work_guard_type(io_context_.get_executor());
    }

    void unblockRunSyncWhenFinishedExecuting()
    {
        work_guard_.reset(); // io_context is now free to return
    }

    // @warning does not stop the game thread.
    void stop()
    {
        server_.close_sessions();
        server_.stop();
    }

private:
    ::rpc::server server_;
    asio::io_context io_context_;
    work_guard_type work_guard_;
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
    static auto wrapSyncCall(asio::io_context& io, FuncT&& functor)
    {
        return
            [&io, functor = std::forward<FuncT>(functor)](Args... args) -> R {
                auto task = std::packaged_task<R()>(
                    [functor = std::move(functor), args...]() {
                        return functor(args...);
                    });
                // Post task and wait for result.
                auto result = task.get_future();
                asio::post(io, moveHandler(task));
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
inline RpcServer::RpcServer(Args&&... args) : server_(std::forward<Args>(args)...), io_context_(), work_guard_(io_context_.get_executor())
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
