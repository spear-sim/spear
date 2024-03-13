//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts>    // std::same_as
#include <cmath>       // std::nan
#include <future>
#include <map>
#include <mutex>
#include <string>
#include <utility>     // std::forward, std::move
#include <vector>

#include <SpCore/Assert.h>
#include <SpCore/BoostAsio.h>

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

//enum class ReturnType
//{
//    Invalid              = -1,
//    Void                 = 0,
//    Bool                 = 1,
//    Int                  = 2,
//    Float                = 3,
//    Double               = 4,
//    String               = 5,
//    VectorDouble         = 6,
//    VectorVectorDouble   = 7,
//    MapStringVectorUInt8 = 8
//};
//
//template <typename TReturn> ReturnType getReturnType() = delete;
//template <>                 ReturnType getReturnType<void>()                                        { return ReturnType::Void; };
//template <>                 ReturnType getReturnType<bool>()                                        { return ReturnType::Bool; };
//template <>                 ReturnType getReturnType<int>()                                         { return ReturnType::Int; };
//template <>                 ReturnType getReturnType<float>()                                       { return ReturnType::Float; };
//template <>                 ReturnType getReturnType<double>()                                      { return ReturnType::Double; };
//template <>                 ReturnType getReturnType<std::string>()                                 { return ReturnType::String; };
//template <>                 ReturnType getReturnType<std::vector<double>>()                         { return ReturnType::VectorDouble; };
//template <>                 ReturnType getReturnType<std::vector<std::vector<double>>>()            { return ReturnType::VectorVectorDouble; };
//template <>                 ReturnType getReturnType<std::map<std::string, std::vector<uint8_t>>>() { return ReturnType::MapStringVectorUInt8; };
//
//struct TaskResult
//{
//    TaskResult() = delete;
//    TaskResult(const std::string& name, ReturnType return_type)
//    {
//        name_ = name;
//        return_type_ = return_type;
//    };
//
//    // Templated interface for getting the underlying result data. This is useful in situations where the caller knows the compile-time return type of the task.
//    template <typename T> T getData() const = delete;
//    template <> bool                                        getData<bool>() const                                        { SP_ASSERT(return_type_ == ReturnType::Bool);                 return data_bool_; };
//    template <> int                                         getData<int>() const                                         { SP_ASSERT(return_type_ == ReturnType::Int);                  return data_int_; };
//    template <> float                                       getData<float>() const                                       { SP_ASSERT(return_type_ == ReturnType::Float);                return data_float_; };
//    template <> double                                      getData<double>() const                                      { SP_ASSERT(return_type_ == ReturnType::Double);               return data_double_; };
//    template <> std::string                                 getData<std::string>() const                                 { SP_ASSERT(return_type_ == ReturnType::String);               return data_string_; };
//    template <> std::vector<double>                         getData<std::vector<double>>() const                         { SP_ASSERT(return_type_ == ReturnType::VectorDouble);         return data_vector_double_; };
//    template <> std::vector<std::vector<double>>            getData<std::vector<std::vector<double>>>() const            { SP_ASSERT(return_type_ == ReturnType::VectorVectorDouble);   return data_vector_vector_double_; };
//    template <> std::map<std::string, std::vector<uint8_t>> getData<std::map<std::string, std::vector<uint8_t>>>() const { SP_ASSERT(return_type_ == ReturnType::MapStringVectorUInt8); return data_map_string_vector_uint8_; };
//
//    // Specialized interface for setting the underlying result data.
//    void setData(const bool& val)                                        { SP_ASSERT(return_type_ == ReturnType::Bool);                 data_bool_                    = val; };
//    void setData(const int& val)                                         { SP_ASSERT(return_type_ == ReturnType::Int);                  data_int_                     = val; };
//    void setData(const float& val)                                       { SP_ASSERT(return_type_ == ReturnType::Float);                data_float_                   = val; };
//    void setData(const double& val)                                      { SP_ASSERT(return_type_ == ReturnType::Double);               data_double_                  = val; };
//    void setData(const std::string& val)                                 { SP_ASSERT(return_type_ == ReturnType::String);               data_string_                  = val; };
//    void setData(const std::vector<double>& val)                         { SP_ASSERT(return_type_ == ReturnType::VectorDouble);         data_vector_double_           = val; };
//    void setData(const std::vector<std::vector<double>>& val)            { SP_ASSERT(return_type_ == ReturnType::VectorVectorDouble);   data_vector_vector_double_    = val; };
//    void setData(const std::map<std::string, std::vector<uint8_t>>& val) { SP_ASSERT(return_type_ == ReturnType::MapStringVectorUInt8); data_map_string_vector_uint8_ = val; };
//
//    std::string name_;
//    ReturnType return_type_ = ReturnType::Invalid;
//
//    bool                                        data_bool_ = false;
//    int                                         data_int_ = -1;
//    float                                       data_float_ = std::nan("");
//    double                                      data_double_ = std::nan("");
//    std::string                                 data_string_;
//    std::vector<double>                         data_vector_double_;
//    std::vector<std::vector<double>>            data_vector_vector_double_;
//    std::map<std::string, std::vector<uint8_t>> data_map_string_vector_uint8_;
//};


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

    //template <typename TReturn>
    //std::future<TReturn>* createFuture(std::future<TReturn>&& future)
    //{
    //    // std::future<TReturn> does not seem to be a supported type for std::any or boost::any. So we
    //    // resort to allocating on the heap and returning a pointer that must be cleaned up later.
    //    return new std::future<TReturn>(std::forward<std::future<TReturn>>(future));
    //}

    //template <typename TReturn>
    //void destroyFuture(std::future<TReturn>* future)
    //{
    //    delete future;
    //}

    //template <typename TReturn>
    //std::future<TReturn>* getFuture(void* future)
    //{
    //    return reinterpret_cast<std::future<TReturn>*>(future);
    //}

    //template <typename TReturn, typename... TArgs>
    //void scheduleTaskNonBlockingImpl(const auto& func, const auto&... args, const FuncInfo<TReturn(*)(TArgs...)>& fi)
    //{
    //    // Note that we capture func and args by value because we want to guarantee that they are
    //    // both still accessible after this scheduleTaskNonBlockingImpl(...) function
    //    // returns. Note also that boost::asio requires us to std::move(task).
    //    auto task = createCopyConstructibleTask(
    //        [func, args...]() -> TReturn {
    //            return func(args...);
    //        });
    //    std::future<TReturn>* future = createFuture(task.get_future());
    //    boost::asio::post(io_context_, std::move(task));

    //    TaskDesc task_desc;
    //    task_desc.name_ = name;
    //    task_desc.return_type_ = getReturnType<TReturn>();
    //    task_desc.future_ = future;
    //    task_descs_.push_back(std::move(task_desc));
    //};

    //template <typename TFunc>
    //auto scheduleTaskNonBlocking(const TFunc& func, const auto&... args)
    //{
    //    scheduleTaskNonBlockingImpl(func, args..., FuncInfo<TFunc>());
    //};

    //template <typename TFunc, typename TReturn, typename... TArgs> requires CCallable<TFunc, TReturn, TArgs...>
    //static auto wrapFuncToExecuteInWorkQueueNonBlockingImpl(WorkQueue& work_queue, TFunc&& func, const FuncInfo<TReturn(*)(TArgs&...)>& fi)
    //{
    //    return [&work_queue, func](const TArgs&... args) -> void {
    //        work_queue.typename scheduleTaskNonBlocking<TFunc>(func, args...);
    //    };
    //}
    // 
    //template <typename TFunc>
    //static auto wrapFuncToExecuteInWorkQueueNonBlocking(WorkQueue& work_queue, TFunc&& func)
    //{
    //    return wrapFuncToExecuteInWorkQueueNonBlockingImpl(work_queue, std::forward<TFunc>(func), FuncInfo<TFunc>());
    //}

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

private:
    std::mutex mutex_;
    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> executor_work_guard_;
};
