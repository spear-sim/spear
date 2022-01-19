// Modified from Carla (https://carla.org) project, thanks to MIT license.

#pragma once

#include <chrono>
#include <future>

#include "MoveHandler.h"

#include "Utils/AsioInclude.h"

#include "Utils/RpcMsgpackInclude.h"

#include "rpc/server.h"

namespace unrealrl
{
namespace rpc
{

//~=============================================================================
// RPC Server class wrapper

/// An RPC server in which functions can be bind to run synchronously or
/// asynchronously.
///
/// Use `AsyncRun` to start the worker threads, and use `SyncRunFor`,
/// `SyncRun()` to run a slice of work in the caller's thread.
///
/// Functions that are bind using `BindAsync` will run asynchronously in the
/// worker threads. Functions that are bind using `BindSync` will run within
/// `SyncRunFor`and `SyncRun()` function.
class Server
{
public:
    using work_guard_type =
        asio::executor_work_guard<asio::io_context::executor_type>;

    template <typename... Args> explicit Server(bool is_sync, Args&&... args);

    template <typename FunctorT>
    void BindSync(const std::string& name, FunctorT&& functor);

    template <typename FunctorT>
    void BindAsync(const std::string& name, FunctorT&& functor);

    void AsyncRun(size_t worker_threads)
    {
        _server.async_run(worker_threads);
    }

    void SyncRun()
    {
        _sync_io_context.restart(); // prepare io_context for subsequent run
        if (_sync_mode)
        {
            _work_guard.reset(
                new work_guard_type(_sync_io_context.get_executor()));
        }
        _sync_io_context.run();
    }

    void ResetWorkGuard()
    {
        _work_guard.reset();
    }

    void SetSyncMode(bool is_sync)
    {
        _sync_mode = is_sync;
    }

    bool GetSyncMode() const
    {
        return _sync_mode;
    }

    /// @warning does not stop the game thread.
    void Stop()
    {
        _server.close_sessions();
        _server.stop();
    }

private:
    asio::io_context _sync_io_context;

    std::unique_ptr<work_guard_type> _work_guard;

    ::rpc::server _server;

    bool _sync_mode;
};

namespace detail
{
template <typename T>
struct FunctionWrapper : FunctionWrapper<decltype(&T::operator())>
{
};

template <typename C, typename R, typename... Args>
struct FunctionWrapper<R (C::*)(Args...)> : FunctionWrapper<R (*)(Args...)>
{
};

template <typename C, typename R, typename... Args>
struct FunctionWrapper<R (C::*)(Args...) const>
    : FunctionWrapper<R (*)(Args...)>
{
};

template <class T> struct FunctionWrapper<T&> : public FunctionWrapper<T>
{
};

template <class T> struct FunctionWrapper<T&&> : public FunctionWrapper<T>
{
};

template <typename R, typename... Args> struct FunctionWrapper<R (*)(Args...)>
{
    /// Wraps @a functor into a function type with equivalent signature. The
    /// wrap function returned. When called, posts @a functor into the
    /// io_context; if the client called this method synchronously, waits for
    /// the posted task to finish, otherwise returns immediately.
    ///
    /// This way, no matter from which thread the wrap function is called, the
    /// @a functor provided is always called from the context of the io_context.
    /// I.e., we can use the io_context to run tasks on a specific thread (e.g.
    /// game thread).
    template <typename FuncT>
    static auto WrapSyncCall(asio::io_context& io, FuncT&& functor)
    {
        return
            [&io, functor = std::forward<FuncT>(functor)](Args... args) -> R {
                auto task = std::packaged_task<R()>(
                    [functor = std::move(functor), args...]() {
                        return functor(args...);
                    });
                // Post task and wait for result.
                auto result = task.get_future();
                asio::post(io, MoveHandler(task));
                return result.get();
            };
    }

    template <typename FuncT> static auto WrapAsyncCall(FuncT&& functor)
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
inline Server::Server(bool is_sync, Args&&... args)
    : _server(std::forward<Args>(args)...), _sync_mode(is_sync)
{
    _server.suppress_exceptions(true);
}

template <typename FunctorT>
inline void Server::BindSync(const std::string& name, FunctorT&& functor)
{
    using Wrapper = detail::FunctionWrapper<FunctorT>;
    _server.bind(name, Wrapper::WrapSyncCall(_sync_io_context,
                                             std::forward<FunctorT>(functor)));
}

template <typename FunctorT>
inline void Server::BindAsync(const std::string& name, FunctorT&& functor)
{
    using Wrapper = detail::FunctionWrapper<FunctorT>;
    _server.bind(name, Wrapper::WrapAsyncCall(std::forward<FunctorT>(functor)));
}

} // namespace rpc
} // namespace unrealrl
