//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <cmath>       // std::nan
#include <concepts>    // std::same_as
#include <future>
#include <mutex>
#include <string>
#include <tuple>
#include <type_traits> // std::is_function, std::is_same_v
#include <typeindex>   // std::type_index
#include <utility>     // std::forward, std::move

#include "CoreUtils/Rpclib.h"
#include "SpEngine/Legacy/BoostAsio.h"

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
struct FunctionWrapper : public FunctionWrapper<decltype(&TClass::operator())> {};

template <typename TClass, typename TReturn, typename... TArgs>
struct FunctionWrapper<TReturn (TClass::*)(TArgs...)> : public FunctionWrapper<TReturn (*)(TArgs...)> {};

template <typename TClass, typename TReturn, typename... TArgs>
struct FunctionWrapper<TReturn (TClass::*)(TArgs...) const> : public FunctionWrapper<TReturn (*)(TArgs...)> {};

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






template <typename TFunc, typename TReturn, typename... TArgs>
concept CCallable =
    requires(TFunc func, TArgs... args) {
        { func(args...) } -> std::same_as<TReturn>;
    };

template <typename TBasicEntryPointBinder>
concept CBasicEntryPointBinder =
    requires(TBasicEntryPointBinder basic_entry_point_binder) {
        { basic_entry_point_binder.template bind("", []() -> void {}) } -> std::same_as<void>;
    };

template <typename TClass>
struct FuncInfo : public FuncInfo<decltype(&TClass::operator())> {};

template <typename TClass, typename TReturn, typename... TArgs>
struct FuncInfo<TReturn(TClass::*)(TArgs...)> : public FuncInfo<TReturn(*)(TArgs...)> {};

template <typename TClass, typename TReturn, typename... TArgs>
struct FuncInfo<TReturn(TClass::*)(TArgs...) const> : public FuncInfo<TReturn(*)(TArgs...)> {};

template <typename TReturn, typename... TArgs>
struct FuncInfo<TReturn(*)(TArgs...)> {};

enum class ReturnType
{
    Invalid              = -1,
    Void                 = 0,
    Bool                 = 1,
    Int                  = 2,
    Float                = 3,
    Double               = 4,
    String               = 5,
    VectorDouble         = 6,
    VectorVectorDouble   = 7,
    MapStringVectorUInt8 = 8
};

template <typename TReturn> ReturnType getReturnType() = delete;
template <>                 ReturnType getReturnType<void>()                                        { return ReturnType::Void; };
template <>                 ReturnType getReturnType<bool>()                                        { return ReturnType::Bool; };
template <>                 ReturnType getReturnType<int>()                                         { return ReturnType::Int; };
template <>                 ReturnType getReturnType<float>()                                       { return ReturnType::Float; };
template <>                 ReturnType getReturnType<double>()                                      { return ReturnType::Double; };
template <>                 ReturnType getReturnType<std::string>()                                 { return ReturnType::String; };
template <>                 ReturnType getReturnType<std::vector<double>>()                         { return ReturnType::VectorDouble; };
template <>                 ReturnType getReturnType<std::vector<std::vector<double>>>()            { return ReturnType::VectorVectorDouble; };
template <>                 ReturnType getReturnType<std::map<std::string, std::vector<uint8_t>>>() { return ReturnType::MapStringVectorUInt8; };

struct TaskResult
{
    TaskResult() = delete;
    TaskResult(const std::string& name, ReturnType return_type)
    {
        name_ = name;
        return_type_ = return_type;
    };

    // Templated interface for getting the underlying result data. This is useful in situations where
    // the caller knows the compile-time return type of the task.
    template <typename T> T getData() const = delete;
    template <> bool                                        getData<bool>() const                                        { assert(return_type_ == ReturnType::Bool);                 return data_bool_; };
    template <> int                                         getData<int>() const                                         { assert(return_type_ == ReturnType::Int);                  return data_int_; };
    template <> float                                       getData<float>() const                                       { assert(return_type_ == ReturnType::Float);                return data_float_; };
    template <> double                                      getData<double>() const                                      { assert(return_type_ == ReturnType::Double);               return data_double_; };
    template <> std::string                                 getData<std::string>() const                                 { assert(return_type_ == ReturnType::String);               return data_string_; };
    template <> std::vector<double>                         getData<std::vector<double>>() const                         { assert(return_type_ == ReturnType::VectorDouble);         return data_vector_double_; };
    template <> std::vector<std::vector<double>>            getData<std::vector<std::vector<double>>>() const            { assert(return_type_ == ReturnType::VectorVectorDouble);   return data_vector_vector_double_; };
    template <> std::map<std::string, std::vector<uint8_t>> getData<std::map<std::string, std::vector<uint8_t>>>() const { assert(return_type_ == ReturnType::MapStringVectorUInt8); return data_map_string_vector_uint8_; };

    // Specialized interface for setting the underlying result data.
    void setData(const bool& val)                                        { assert(return_type_ == ReturnType::Bool);                 data_bool_                    = val; };
    void setData(const int& val)                                         { assert(return_type_ == ReturnType::Int);                  data_int_                     = val; };
    void setData(const float& val)                                       { assert(return_type_ == ReturnType::Float);                data_float_                   = val; };
    void setData(const double& val)                                      { assert(return_type_ == ReturnType::Double);               data_double_                  = val; };
    void setData(const std::string& val)                                 { assert(return_type_ == ReturnType::String);               data_string_                  = val; };
    void setData(const std::vector<double>& val)                         { assert(return_type_ == ReturnType::VectorDouble);         data_vector_double_           = val; };
    void setData(const std::vector<std::vector<double>>& val)            { assert(return_type_ == ReturnType::VectorVectorDouble);   data_vector_vector_double_    = val; };
    void setData(const std::map<std::string, std::vector<uint8_t>>& val) { assert(return_type_ == ReturnType::MapStringVectorUInt8); data_map_string_vector_uint8_ = val; };

    std::string name_;
    ReturnType return_type_ = ReturnType::Invalid;

    bool                                        data_bool_ = false;
    int                                         data_int_ = -1;
    float                                       data_float_ = std::nan("");
    double                                      data_double_ = std::nan("");
    std::string                                 data_string_;
    std::vector<double>                         data_vector_double_;
    std::vector<std::vector<double>>            data_vector_vector_double_;
    std::map<std::string, std::vector<uint8_t>> data_map_string_vector_uint8_;
};

template <typename TReturn>
struct CopyConstructibleTask : std::packaged_task<TReturn()>
{
    CopyConstructibleTask(auto&& func) : std::packaged_task<TReturn()>(std::forward<decltype(func)>(func)) {};
};

template <typename TFunc, typename TReturn, typename... TArgs> requires CCallable<TFunc, TReturn, TArgs...>
static CopyConstructibleTask<TReturn> createCopyConstructibleTaskImpl(TFunc&& func, const FuncInfo<TReturn(*)(TArgs...)>& fi)
{
    return CopyConstructibleTask<TReturn>(std::forward<decltype(func)>(func));
}

template <typename TFunc>
static auto createCopyConstructibleTask(TFunc&& func)
{
    return createCopyConstructibleTaskImpl(std::forward<TFunc>(func), FuncInfo<TFunc>());
}

template <typename TReturn>
std::future<TReturn>* createFuture(std::future<TReturn>&& future)
{
    // std::future<TReturn> does not seem to be a supported type for std::any or boost::any. So we
    // resort to allocating on the heap and returning a pointer that must be cleaned up later.
    return new std::future<TReturn>(std::forward<std::future<TReturn>>(future));
}

template <typename TReturn>
void destroyFuture(std::future<TReturn>* future)
{
    delete future;
}

template <typename TReturn>
std::future<TReturn>* getFuture(void* future)
{
    return reinterpret_cast<std::future<TReturn>*>(future);
}

// Specialized interface for casting a future to the appropriate type, and applying it to a function,
// given a void* pointer to the future. We write this function to take a function as input so we only
// need to have the ugly type-conditioned switch statement in one place, and the rest of our code can
// assume a strongly typed interface.
void applyFutureToFunc(void* future, ReturnType return_type, const auto& func)
{
    switch (return_type) {
        case ReturnType::Void:                 func(getFuture<void>(future));                                        break;
        case ReturnType::Bool:                 func(getFuture<bool>(future));                                        break;
        case ReturnType::Int:                  func(getFuture<int>(future));                                         break;
        case ReturnType::Float:                func(getFuture<float>(future));                                       break;
        case ReturnType::Double:               func(getFuture<double>(future));                                      break;
        case ReturnType::String:               func(getFuture<std::string>(future));                                 break;
        case ReturnType::VectorDouble:         func(getFuture<std::vector<double>>(future));                         break;
        case ReturnType::VectorVectorDouble:   func(getFuture<std::vector<std::vector<double>>>(future));            break;
        case ReturnType::MapStringVectorUInt8: func(getFuture<std::map<std::string, std::vector<uint8_t>>>(future)); break;
        default:                               assert(false);                                                        break;
    }
}

// Generic interface for destroying a future, given a void* pointer to it.
void destroyFuture(void* future, ReturnType return_type)
{
    applyFutureToFunc(future, return_type, [](auto future) -> void {
        destroyFuture(future);
    });
}

// Generic interface for updating a TaskResult object, given a void* pointer to it.
void updateTaskResultBlocking(TaskResult& task_result, void* future)
{
    applyFutureToFunc(future, task_result.return_type_, [&task_result]<typename TReturn>(std::future<TReturn>* future) -> void {
        if constexpr (std::is_same_v<TReturn, void>) {
            future->get();
        } else {
            task_result.setData(future->get());
        }
    });
}

template <typename TFunc, typename TReturn, typename... TArgs> requires CCallable<TFunc, TReturn, TArgs...>
static TReturn scheduleAndExecuteTaskBlockingImpl(boost::asio::io_context& io_context, const TFunc& func, const FuncInfo<TReturn(*)(TArgs&...)>& fi, auto&... args)
{
    // Note that we capture func and args by value because we want to guarantee that they are both
    // still accessible after this scheduleAndExecuteTaskBlockingImpl(...) function returns. Strictly
    // speaking, this guarantee is not necessary for this blocking implementation, but we capture
    // by value anyway for consistency with our non-blocking implementation.

    // We need to declare our lambda to be mutable here because we expect some of our args to be
    // std::vectors, and we want the tasks we're executing to be able to create std::spans of these
    // vectors. But creating an std::span requires a non-const reference to a vector.
    auto task = createCopyConstructibleTask(
        [func, args...]() mutable -> TReturn {
            return func(args...);
        });

    std::future<TReturn>* future = createFuture(task.get_future());
    boost::asio::post(io_context, std::move(task));
    ReturnType return_type = getReturnType<TReturn>();
    TaskResult task_result("my_func", return_type);
    updateTaskResultBlocking(task_result, future);
    destroyFuture(future, return_type);
    future = nullptr;

    if constexpr (std::is_same_v<TReturn, void>) {
        return;
    } else {
        return task_result.getData<TReturn>();
    }
}

template <typename TFunc>
static auto scheduleAndExecuteTaskBlocking(boost::asio::io_context& io_context, const TFunc& func, auto&... args)
{
    return scheduleAndExecuteTaskBlockingImpl(io_context, func, FuncInfo<TFunc>(), args...);
}

template <typename TFunc, typename TReturn, typename... TArgs> requires CCallable<TFunc, TReturn, TArgs...>
static auto wrapFuncToExecuteInWorkQueueBlockingImpl(boost::asio::io_context& io_context, TFunc&& func, const FuncInfo<TReturn(*)(TArgs&...)>& fi)
{
    // Note that we capture func by value because we want to guarantee that func is still
    // accessible after this wrapFuncToExecuteInWorkQueueBlocking(...) function returns.

    // Note also that we assume that the user's function accepts arguments by const reference.
    // This will avoid an unnecessary copy when the surrounding code (e.g., the RPC server)
    // calls the function returned here. Deeper inside our implementation, we will eventually
    // copy these arguments so they are guaranteed to be accessible when we eventually execute
    // the user's function, but we want to avoid copying wherever possible.
    return [&io_context, func](TArgs&... args) -> TReturn {
        return scheduleAndExecuteTaskBlocking(io_context, func, args...);
    };
}

template <typename TFunc>
static auto wrapFuncToExecuteInWorkQueueBlocking(boost::asio::io_context& io_context, TFunc&& func)
{
    return wrapFuncToExecuteInWorkQueueBlockingImpl(io_context, std::forward<TFunc>(func), FuncInfo<TFunc>());
}

static void bindEntryPointImpl(CBasicEntryPointBinder auto& basic_entry_point_binder, const std::string& name, auto&& func)
{
    basic_entry_point_binder.bind(name, std::forward<decltype(func)>(func));
}

static void bindEntryPoint(CBasicEntryPointBinder auto& basic_entry_point_binder, const std::string& name, auto&& func)
{
    bindEntryPointImpl(basic_entry_point_binder, name, std::forward<decltype(func)>(func));
}

inline void RpcServer::bindSync(const std::string& name, auto&& func)
{
    bindEntryPoint(server_, name, wrapFuncToExecuteInWorkQueueBlocking(io_context_, std::forward<decltype(func)>(func)));
}

template <typename TFunctor>
inline void RpcServer::bindAsync(const std::string& name, TFunctor&& functor)
{
    server_.bind(name, detail::FunctionWrapper<TFunctor>::wrapAsync(std::forward<TFunctor>(functor)));
}
