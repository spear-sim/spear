// This is borrowed from https://github.com/adamrehn/ue4-grpc-demo

#pragma once
#include "Async/Async.h"

// The code in this file is adapted from the templates in:
//<Engine/Source/Runtime/Core/Public/Async/Async.h>

// Modified version of TAsyncGraphTask that accepts a user-specified thread to
// run on
template <typename ResultType>
class TNamedThreadGraphTask : public FAsyncGraphTaskBase
{
public:
    TNamedThreadGraphTask(ENamedThreads::Type InDesiredThread,
                          TFunction<ResultType()>&& InFunction,
                          TPromise<ResultType>&& InPromise)
        : DesiredThread(InDesiredThread), Function(MoveTemp(InFunction)),
          Promise(MoveTemp(InPromise))
    {
    }

    void DoTask(ENamedThreads::Type CurrentThread,
                const FGraphEventRef& MyCompletionGraphEvent)
    {
        SetPromise(Promise, Function);
    }

    ENamedThreads::Type GetDesiredThread()
    {
        return DesiredThread;
    }

    TFuture<ResultType> GetFuture()
    {
        return Promise.GetFuture();
    }

private:
    ENamedThreads::Type DesiredThread;
    TFunction<ResultType()> Function;
    TPromise<ResultType> Promise;
};

// Modified version of Async() that utilises the template class above
template <typename ResultType>
TFuture<ResultType> AsyncNamed(ENamedThreads::Type DesiredThread,
                               TFunction<ResultType()> Function)
{
    TPromise<ResultType> Promise;
    TFuture<ResultType> Future = Promise.GetFuture();
    TGraphTask<TNamedThreadGraphTask<ResultType>>::CreateTask()
        .ConstructAndDispatchWhenReady(DesiredThread, MoveTemp(Function),
                                       MoveTemp(Promise));
    return MoveTemp(Future);
}
