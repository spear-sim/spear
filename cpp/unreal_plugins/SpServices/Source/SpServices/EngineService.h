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
#include "SpServices/ServiceUtils.h"
#include "SpServices/WorkQueue.h"

#include "EngineService.generated.h"

UENUM()
enum class EFrameState
{
    Invalid           = -1,
    Idle              = 0,
    RequestPreTick    = 1,
    ExecutingPreTick  = 2,
    ExecutingTick     = 3,
    ExecutingPostTick = 4,
    Closing           = 5
};

USTRUCT()
struct FFrameState
{
    GENERATED_BODY()
    UPROPERTY()
    EFrameState Enum = EFrameState::Invalid;
    SP_DECLARE_ENUM_PROPERTY(EFrameState, Enum);
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

        frame_state_ = EFrameState::Idle;

        entry_point_binder_->bind("engine_service.get_frame_state", [this]() -> std::string {
            return Unreal::getEnumValueAsString<FFrameState>(frame_state_.load());
        });

        entry_point_binder_->bind("engine_service.get_world", [this]() -> uint64 {
            return ServiceUtils::toUInt64(world_.load());
        });

        entry_point_binder_->bind("engine_service.get_byte_order", []() -> std::string {
            SP_ASSERT(BOOST_ENDIAN_BIG_BYTE + BOOST_ENDIAN_LITTLE_BYTE == 1);
            if (BOOST_ENDIAN_BIG_BYTE) {
                return "big";
            } else if (BOOST_ENDIAN_LITTLE_BYTE) {
                return "little";
            } else {
                return "";
            }
        });

        entry_point_binder_->bind("engine_service.begin_tick", [this]() -> void {
            
            // We need to lock frame_state_mutex_ here, because the game thread might call close() any time
            // before, while, or after executing this function. If close() is executed after we check if
            // frame_state_ == EFrameState::Idle but before we set frame_state_ = EFrameState::RequestPreTick,
            // then we will deadlock because frame_state_executing_pre_tick_future_.wait() will never return.
            // We avoid this problematic case by locking frame_state_mutex_.
            frame_state_mutex_.lock();
            {
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
            frame_state_mutex_.unlock();

            if (frame_state_ == EFrameState::RequestPreTick) {
                // Wait here until beginFrameHandler() or close() updates frame_state_ and calls frame_state_executing_pre_tick_promise_.set_value().
                frame_state_executing_pre_tick_future_.wait();
                SP_ASSERT(frame_state_ == EFrameState::ExecutingPreTick || frame_state_ == EFrameState::Closing);
            }
        });

        entry_point_binder_->bind("engine_service.tick", [this]() -> void {
            SP_ASSERT(frame_state_ == EFrameState::ExecutingPreTick);

            // Allow beginFrameHandler() to finish executing.
            work_queue_.reset();

            // Wait here until endFrameHandler() updates frame_state_ and calls frame_state_executing_post_tick_promise_.set_value().
            frame_state_executing_post_tick_future_.wait();
            SP_ASSERT(frame_state_ == EFrameState::ExecutingPostTick);
        });

        entry_point_binder_->bind("engine_service.end_tick", [this]() -> void {
            SP_ASSERT(frame_state_ == EFrameState::ExecutingPostTick);

            // Allow endFrameHandler() to finish executing.
            work_queue_.reset();

            // Wait here until endFrameHandler() updates frame_state_ and calls frame_state_idle_promise_.set_value().
            frame_state_idle_future_.wait();
            SP_ASSERT(frame_state_ == EFrameState::Idle);
        });

        entry_point_binder_->bind("engine_service.request_close", []() -> void {
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
        entry_point_binder_->bind(service_name + "." + func_name, func);
    }

    void bindFuncUnreal(const std::string& service_name, const std::string& func_name, const auto& func)
    {
        entry_point_binder_->bind(service_name + "." + func_name, WorkQueue::wrapFuncToExecuteInWorkQueueBlocking(work_queue_, func));
    }

    void close()
    {
        // We need to lock frame_state_mutex_ here, because the RPC worker thread might call begin_tick() any
        // time before, while, or after executing this function. If begin_tick() is executed after we set
        // previous_frame_state = frame_state_ but before we set frame_state_ = EFrameState::Closing, then
        // begin_tick() will deadlock because frame_state_executing_pre_tick_future_.wait() will never return.
        // We avoid this problematic case by locking frame_state_mutex_.
        EFrameState previous_frame_state = EFrameState::Invalid;
        frame_state_mutex_.lock();
        {
            SP_ASSERT(frame_state_ == EFrameState::Idle || frame_state_ == EFrameState::RequestPreTick);
            previous_frame_state = frame_state_;
            frame_state_ = EFrameState::Closing;
        }
        frame_state_mutex_.unlock();

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
        }
    }

    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(world);

        SP_LOG("World name: ", Unreal::toStdString(world->GetName()));

        if (world == world_) {
            SP_LOG("Clearing cached world...");
            world_ = nullptr;
        }
    }

    void beginFrameHandler()
    {
        if (frame_state_ == EFrameState::RequestPreTick) {
            // Allow begin_tick() to finish executing. There is no need to lock frame_state_mutex_ here,
            // because if frame_state_ == EFrameState::RequestPreTick, then we know the RPC worker thread is
            // currently waiting in begin_tick() at a point where it will not attempt to make any further
            // modifications to frame_state_.
            frame_state_ = EFrameState::ExecutingPreTick;
            frame_state_executing_pre_tick_promise_.set_value();

            // Execute all pre-tick work and wait until tick() calls work_queue_.reset().
            work_queue_.run();

            // Update frame state.
            frame_state_ = EFrameState::ExecutingTick;
        }
    }

    void endFrameHandler()
    {
        if (frame_state_ == EFrameState::ExecutingTick) {
            // Allow tick() to finish executing. There is no need to lock frame_state_mutex_ here, because
            // if frame_state_ == EFrameState::ExecutingTick, then we know the RPC worker thread is currently
            // waiting in tick(), and tick() doesn't modify frame_state_.
            frame_state_ = EFrameState::ExecutingPostTick;
            frame_state_executing_post_tick_promise_.set_value();

            // Execute all pre-tick work and wait until end_tick() calls work_queue_.reset().
            work_queue_.run();

            // Allow end_tick() to finish executing.
            frame_state_ = EFrameState::Idle;
            frame_state_idle_promise_.set_value();
        }
    }

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;
    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;

    std::atomic<UWorld*> world_ = nullptr;

    TEntryPointBinder* entry_point_binder_ = nullptr;
    WorkQueue work_queue_;

    std::atomic<EFrameState> frame_state_ = EFrameState::Invalid;
    std::mutex frame_state_mutex_;

    std::promise<void> frame_state_idle_promise_;
    std::promise<void> frame_state_executing_pre_tick_promise_;
    std::promise<void> frame_state_executing_post_tick_promise_;

    std::future<void> frame_state_idle_future_;
    std::future<void> frame_state_executing_pre_tick_future_;
    std::future<void> frame_state_executing_post_tick_future_;
};
