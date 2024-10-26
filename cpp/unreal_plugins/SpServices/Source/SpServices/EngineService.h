//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <atomic>
#include <future> // std::promise, std::future
#include <mutex>
#include <string>

#include <Containers/UnrealString.h>     // FString::operator*
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Misc/CoreDelegates.h>
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS, UENUM, UPROPERTY, USTRUCT

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/FuncInfo.h"
#include "SpServices/ServiceUtils.h"
#include "SpServices/WorkQueue.h"

#include "EngineService.generated.h"

UENUM()
enum class EFrameState
{
    NotInitialized    = 0,
    Idle              = 1,
    RequestPreTick    = 2,
    ExecutingPreTick  = 3,
    ExecutingTick     = 4,
    ExecutingPostTick = 5,
    Closing           = 6,
    Error             = 7
};

template <CEntryPointBinder TEntryPointBinder>
class EngineService {
public:
    EngineService() = delete;
    EngineService(TEntryPointBinder* entry_point_binder)
    {
        SP_ASSERT(entry_point_binder);

        entry_point_binder_ = entry_point_binder;

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &EngineService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &EngineService::worldCleanupHandler);
        begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &EngineService::beginFrameHandler);
        end_frame_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &EngineService::endFrameHandler);

        entry_point_binder_->bind("engine_service.get_world", [this]() -> uint64 {
            return ServiceUtils::toUInt64(world_.load());
        });

        entry_point_binder_->bind("engine_service.get_frame_state", [this]() -> std::string {
            return Unreal::getStringFromEnumValue<EFrameState>(frame_state_.load());
        });

        entry_point_binder_->bind("engine_service.begin_tick", [this]() -> void {

            // If we're in an error state, return immediately.
            if (frame_state_ == EFrameState::Error) {
                return;
            }

            {
                std::lock_guard<std::mutex> lock(frame_state_mutex_);

                SP_ASSERT(frame_state_ == EFrameState::Idle || frame_state_ == EFrameState::Closing);

                if (frame_state_ == EFrameState::Idle) {
                    // Reset promises and futures.
                    frame_state_idle_promise_ = std::promise<void>();
                    frame_state_executing_pre_tick_promise_ = std::promise<void>();
                    frame_state_executing_post_tick_promise_ = std::promise<void>();

                    frame_state_idle_future_ = frame_state_idle_promise_.get_future();
                    frame_state_executing_pre_tick_future_ = frame_state_executing_pre_tick_promise_.get_future();
                    frame_state_executing_post_tick_future_ = frame_state_executing_post_tick_promise_.get_future();

                    // Allow beginFrameHandler() to start executing.
                    frame_state_ = EFrameState::RequestPreTick;
                }
            }

            if (frame_state_ == EFrameState::RequestPreTick) {
                // Wait here until beginFrameHandler() or close() updates frame_state_ and calls frame_state_executing_pre_tick_promise_.set_value().
                frame_state_executing_pre_tick_future_.wait();
                SP_ASSERT(frame_state_ == EFrameState::ExecutingPreTick || frame_state_ == EFrameState::Closing);
            }
        });

        entry_point_binder_->bind("engine_service.tick", [this]() -> void {

            // If we're in an error state, return immediately.
            if (frame_state_ == EFrameState::Error) {
                return;
            }

            SP_ASSERT(frame_state_ == EFrameState::ExecutingPreTick);

            // Allow beginFrameHandler() to finish executing.
            work_queue_.reset();

            // Wait here until endFrameHandler() updates frame_state_ and calls frame_state_executing_post_tick_promise_.set_value().
            frame_state_executing_post_tick_future_.wait();
            SP_ASSERT(frame_state_ == EFrameState::ExecutingPostTick);
        });

        entry_point_binder_->bind("engine_service.end_tick", [this]() -> void {

            // If we're in an error state, return immediately.
            if (frame_state_ == EFrameState::Error) {
                return;
            }

            SP_ASSERT(frame_state_ == EFrameState::ExecutingPostTick);

            // Allow endFrameHandler() to finish executing.
            work_queue_.reset();

            // Wait here until endFrameHandler() updates frame_state_ and calls frame_state_idle_promise_.set_value().
            frame_state_idle_future_.wait();
            SP_ASSERT(frame_state_ == EFrameState::Idle);
        });

        entry_point_binder_->bind("engine_service.request_exit", []() -> void {
            bool immediate_shutdown = false;
            FGenericPlatformMisc::RequestExit(immediate_shutdown);
        });
    }

    ~EngineService()
    {
        FCoreDelegates::OnEndFrame.Remove(end_frame_handle_);
        FCoreDelegates::OnBeginFrame.Remove(begin_frame_handle_);

        end_frame_handle_.Reset();
        begin_frame_handle_.Reset();

        SP_ASSERT(entry_point_binder_);
        entry_point_binder_ = nullptr;
    }

    void bindFuncNoUnreal(const std::string& service_name, const std::string& func_name, const auto& func)
    {
        std::string long_func_name = service_name + "." + func_name;
        entry_point_binder_->bind(long_func_name, wrapFuncToExecuteInTryCatchBlock(long_func_name, func));
    }

    void bindFuncUnreal(const std::string& service_name, const std::string& func_name, const auto& func)
    {
        std::string long_func_name = service_name + "." + func_name;
        entry_point_binder_->bind(service_name + "." + func_name, WorkQueue::wrapFuncToExecuteInWorkQueueBlocking(work_queue_, long_func_name, func));
    }

    void close()
    {
        std::lock_guard<std::mutex> lock(frame_state_mutex_);
        SP_ASSERT(frame_state_ == EFrameState::NotInitialized || frame_state_ == EFrameState::Idle || frame_state_ == EFrameState::RequestPreTick);

        EFrameState previous_frame_state = EFrameState::NotInitialized;
        frame_state_ = EFrameState::Closing;

        if (previous_frame_state == EFrameState::RequestPreTick) {
            // Allow begin_tick() to finish executing.
            frame_state_executing_pre_tick_promise_.set_value();
        }
    }

private:
    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(world);

        SP_LOG("World name: ", Unreal::toStdString(world->GetName()));

        if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
            SP_LOG("Caching world...");
            SP_ASSERT(!world_);
            world_ = world;
            world_begin_play_handle_ = world_.load()->OnWorldBeginPlay.AddRaw(this, &EngineService::worldBeginPlayHandler);
        }
    }


    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(world);

        SP_LOG("World name: ", Unreal::toStdString(world->GetName()));

        frame_state_ = EFrameState::NotInitialized;

        if (world == world_) {
            SP_LOG("Clearing cached world...");
            world_.load()->OnWorldBeginPlay.Remove(world_begin_play_handle_);
            world_begin_play_handle_.Reset();
            world_ = nullptr;
        }
    }

    void worldBeginPlayHandler()
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(world_);

        work_queue_.initialize();
        frame_state_ = EFrameState::Idle;
    }

    void beginFrameHandler()
    {
        std::lock_guard<std::mutex> lock(frame_state_mutex_);

        if (frame_state_ == EFrameState::RequestPreTick) {

            // Allow begin_tick() to finish executing.
            frame_state_ = EFrameState::ExecutingPreTick;
            frame_state_executing_pre_tick_promise_.set_value();

            // Execute all pre-tick work and wait until tick() calls work_queue_.reset().
            try {
                work_queue_.run();

            // In case of an exception, set the frame state to be in an error state and return.
            } catch(const std::exception& e) {
                SP_LOG("Caught exception when executing beginFrameHandler(): ", e.what());
                frame_state_ = EFrameState::Error;
                return;
            } catch(...) {
                SP_LOG("Caught unknown exception when executing beginFrameHandler().");
                frame_state_ = EFrameState::Error;
                return;
            }

            // Update frame state.
            frame_state_ = EFrameState::ExecutingTick;
        }
    }

    void endFrameHandler()
    {
        std::lock_guard<std::mutex> lock(frame_state_mutex_);

        if (frame_state_ == EFrameState::ExecutingTick) {

            // Allow tick() to finish executing.
            frame_state_ = EFrameState::ExecutingPostTick;
            frame_state_executing_post_tick_promise_.set_value();

            // Execute all pre-tick work and wait until end_tick() calls work_queue_.reset().
            try {
                work_queue_.run();

            // In case of an exception, set the frame state to be in an error state and return.
            } catch(const std::exception& e) {
                SP_LOG("Caught exception when executing endFrameHandler(): ", e.what());
                frame_state_ = EFrameState::Error;
                return;
            } catch(...) {
                SP_LOG("Caught unknown exception when executing endFrameHandler().");
                frame_state_ = EFrameState::Error;
                return;
            }

            // Allow end_tick() to finish executing.
            frame_state_ = EFrameState::Idle;
            frame_state_idle_promise_.set_value();
        }
    }

    template <typename TFunc>
    auto wrapFuncToExecuteInTryCatchBlock(const std::string& long_func_name, const TFunc& func)
    {
        return wrapFuncToExecuteInTryCatchBlockImpl(long_func_name, func, FuncInfo<TFunc>());
    }

    template <typename TFunc, typename TReturn, typename... TArgs> requires
        CFuncReturnsAndIsCallableWithArgs<TFunc, TReturn, TArgs&...>
    auto wrapFuncToExecuteInTryCatchBlockImpl(const std::string& long_func_name, const TFunc& func, const FuncInfo<TReturn(*)(TArgs...)>& fi)
    {
        // The lambda returned here is typically bound to a specific RPC entry point and called from a worker
        // thread by the RPC server.

        // Note that we capture long_func_name and func by value because we want to guarantee that func is still
        // accessible after this wrapFuncToHandleExceptionsImpl(...) function returns.

        // Note also that we assume that the user's function always accepts all arguments by non-const
        // reference. This will avoid unnecessary copying when we eventually call the user's function. We
        // can't assume the user's function accepts arguments by const reference, because the user's function
        // might want to modify the arguments, e.g., when a user function resolves pointers to shared memory
        // for an input SpFuncPackedArray& before forwarding it to an inner function.

        return [this, long_func_name, func](TArgs&... args) -> TReturn {

            try {
                return func(args...);

            } catch (const std::exception& e) {
                SP_LOG("Caught exception when executing ", long_func_name, ": ", e.what());
                work_queue_.reset();
                {
                    std::lock_guard<std::mutex> lock(frame_state_mutex_);
                    frame_state_ = EFrameState::Error;
                }
                return TReturn();
            } catch(...) {
                SP_LOG("Caught unknown exception when executing ", long_func_name, ".");
                work_queue_.reset();
                {
                    std::lock_guard<std::mutex> lock(frame_state_mutex_);
                    frame_state_ = EFrameState::Error;
                }
                return TReturn();
            }
        };
    }

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;
    FDelegateHandle world_begin_play_handle_;
    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;

    std::atomic<UWorld*> world_ = nullptr;

    TEntryPointBinder* entry_point_binder_ = nullptr;
    WorkQueue work_queue_;

    std::atomic<EFrameState> frame_state_ = EFrameState::NotInitialized;
    std::mutex frame_state_mutex_;

    std::promise<void> frame_state_idle_promise_;
    std::promise<void> frame_state_executing_pre_tick_promise_;
    std::promise<void> frame_state_executing_post_tick_promise_;

    std::future<void> frame_state_idle_future_;
    std::future<void> frame_state_executing_pre_tick_future_;
    std::future<void> frame_state_executing_post_tick_future_;
};
