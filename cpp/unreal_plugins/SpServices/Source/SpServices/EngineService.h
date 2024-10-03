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

#include "SpCore/Assert.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/WorkQueue.h"

#if !WITH_EDITOR
    #include <HAL/IConsoleManager.h>

    #include "SpCore/Unreal.h"
#endif

enum class FrameState : int8_t
{
    Invalid           = -1,
    Idle              = 0,
    RequestPreTick    = 1,
    ExecutingPreTick  = 2,
    ExecutingTick     = 3,
    ExecutingPostTick = 4,
    Closing           = 5
};

template <CEntryPointBinder TEntryPointBinder>
class EngineService {
public:
    EngineService() = delete;
    EngineService(TEntryPointBinder* entry_point_binder)
    {
        SP_ASSERT(entry_point_binder);

        entry_point_binder_ = entry_point_binder;

        begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &EngineService::beginFrameHandler);
        end_frame_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &EngineService::endFrameHandler);

        frame_state_ = FrameState::Idle;

        // To work around a platform-specific rendering bug, we explicitly disable Lumen and then
        // conditionally re-enable it the first time beginFrameHandler() gets called. We have not seen this
        // bug on Windows, but we have seen it macOS, where it appears to only affect standalone shipping
        // builds. Since we're not exactly sure what configurations are affected, we choose to implement the
        // workaround on all standalone builds.
        #if !WITH_EDITOR // defined in an auto-generated header
            r_lumen_diffuse_indirect_allow_cvar_ = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.Lumen.DiffuseIndirect.Allow"));
            r_lumen_diffuse_indirect_allow_cvar_initial_value_ = r_lumen_diffuse_indirect_allow_cvar_->GetInt();
            r_lumen_diffuse_indirect_allow_cvar_->Set(0);
        #endif

        entry_point_binder_->bind("engine_service.ping", []() -> std::string {
            return "received a call to engine_service.ping";
        });

        entry_point_binder_->bind("engine_service.request_close", []() -> void {
            bool immediate_shutdown = false;
            FGenericPlatformMisc::RequestExit(immediate_shutdown);
        });

        entry_point_binder_->bind("engine_service.begin_tick", [this]() -> void {
            
            // We need to lock frame_state_mutex_ here, because the game thread might call close() any time
            // before, while, or after executing this function. If close() is executed after we check if
            // frame_state_ == FrameState::Idle but before we set frame_state_ = FrameState::RequestPreTick,
            // then we will deadlock because frame_state_executing_pre_tick_future_.wait() will never return.
            // We avoid this problematic case by locking frame_state_mutex_.
            frame_state_mutex_.lock();
            {
                SP_ASSERT(frame_state_ == FrameState::Idle || frame_state_ == FrameState::Closing);

                if (frame_state_ == FrameState::Idle) {
                    // Reset promises and futures.
                    frame_state_idle_promise_ = std::promise<void>();
                    frame_state_executing_pre_tick_promise_ = std::promise<void>();
                    frame_state_executing_post_tick_promise_ = std::promise<void>();

                    frame_state_idle_future_ = frame_state_idle_promise_.get_future();
                    frame_state_executing_pre_tick_future_ = frame_state_executing_pre_tick_promise_.get_future();
                    frame_state_executing_post_tick_future_ = frame_state_executing_post_tick_promise_.get_future();

                    // Allow beginFrameHandler() to start executing.
                    frame_state_ = FrameState::RequestPreTick;
                }
            }
            frame_state_mutex_.unlock();

            if (frame_state_ == FrameState::RequestPreTick) {
                // Wait here until beginFrameHandler() or close() updates frame_state_ and calls frame_state_executing_pre_tick_promise_.set_value().
                frame_state_executing_pre_tick_future_.wait();
                SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick || frame_state_ == FrameState::Closing);
            }
        });

        entry_point_binder_->bind("engine_service.tick", [this]() -> void {
            SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);

            // Allow beginFrameHandler() to finish executing.
            work_queue_.reset();

            // Wait here until endFrameHandler() updates frame_state_ and calls frame_state_executing_post_tick_promise_.set_value().
            frame_state_executing_post_tick_future_.wait();
            SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        });

        entry_point_binder_->bind("engine_service.end_tick", [this]() -> void {
            SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);

            // Allow endFrameHandler() to finish executing.
            work_queue_.reset();

            // Wait here until endFrameHandler() updates frame_state_ and calls frame_state_idle_promise_.set_value().
            frame_state_idle_future_.wait();
            SP_ASSERT(frame_state_ == FrameState::Idle);
        });

        entry_point_binder_->bind("engine_service.get_byte_order", []() -> std::string {
            uint32_t dummy = 0x01020304;
            return (reinterpret_cast<uint8_t*>(&dummy)[3] == 1) ? "little" : "big";
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
        // previous_frame_state = frame_state_ but before we set frame_state_ = FrameState::Closing, then
        // begin_tick() will deadlock because frame_state_executing_pre_tick_future_.wait() will never return.
        // We avoid this problematic case by locking frame_state_mutex_.
        FrameState previous_frame_state = FrameState::Invalid;
        frame_state_mutex_.lock();
        {
            SP_ASSERT(frame_state_ == FrameState::Idle || frame_state_ == FrameState::RequestPreTick);
            previous_frame_state = frame_state_;
            frame_state_ = FrameState::Closing;
        }
        frame_state_mutex_.unlock();

        if (previous_frame_state == FrameState::RequestPreTick) {
            // Allow begin_tick() to finish executing.
            frame_state_executing_pre_tick_promise_.set_value();
        }
    }

private:
    void beginFrameHandler()
    {
        // Works around a platform-specific rendering bug. See comment in the constructor above.
        #if !WITH_EDITOR
            static bool once = false;
            if (!once) {
                r_lumen_diffuse_indirect_allow_cvar_->Set(r_lumen_diffuse_indirect_allow_cvar_initial_value_);
                once = true;
            }
        #endif

        if (frame_state_ == FrameState::RequestPreTick) {
            // Allow begin_tick() to finish executing. There is no need to lock frame_state_mutex_ here,
            // because if frame_state_ == FrameState::RequestPreTick, then we know the RPC worker thread is
            // currently waiting in begin_tick() at a point where it will not attempt to make any further
            // modifications to frame_state_.
            frame_state_ = FrameState::ExecutingPreTick;
            frame_state_executing_pre_tick_promise_.set_value();

            // Execute all pre-tick work and wait until tick() calls work_queue_.reset().
            work_queue_.run();

            // Update frame state.
            frame_state_ = FrameState::ExecutingTick;
        }
    }

    void endFrameHandler()
    {
        if (frame_state_ == FrameState::ExecutingTick) {
            // Allow tick() to finish executing. There is no need to lock frame_state_mutex_ here, because
            // if frame_state_ == FrameState::ExecutingTick, then we know the RPC worker thread is currently
            // waiting in tick(), and tick() doesn't modify frame_state_.
            frame_state_ = FrameState::ExecutingPostTick;
            frame_state_executing_post_tick_promise_.set_value();

            // Execute all pre-tick work and wait until end_tick() calls work_queue_.reset().
            work_queue_.run();

            // Allow end_tick() to finish executing.
            frame_state_ = FrameState::Idle;
            frame_state_idle_promise_.set_value();
        }
    }

    TEntryPointBinder* entry_point_binder_ = nullptr;
    WorkQueue work_queue_;

    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;

    std::atomic<FrameState> frame_state_ = FrameState::Invalid;
    std::mutex frame_state_mutex_;

    std::promise<void> frame_state_idle_promise_;
    std::promise<void> frame_state_executing_pre_tick_promise_;
    std::promise<void> frame_state_executing_post_tick_promise_;

    std::future<void> frame_state_idle_future_;
    std::future<void> frame_state_executing_pre_tick_future_;
    std::future<void> frame_state_executing_post_tick_future_;

    #if !WITH_EDITOR
        IConsoleVariable* r_lumen_diffuse_indirect_allow_cvar_ = nullptr;        
        int r_lumen_diffuse_indirect_allow_cvar_initial_value_ = -1;
    #endif
};
