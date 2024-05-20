//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <stdint.h> // uint8_t

#include <atomic>
#include <concepts> // std::same_as
#include <future>   // std::promise, std::future
#include <string>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Misc/CoreDelegates.h>

#include "SpCore/Assert.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/WorkQueue.h"

// Different possible frame states for thread synchronization
enum class FrameState : uint8_t
{
    Idle,
    RequestPreTick,
    ExecutingPreTick,
    ExecutingTick,
    ExecutingPostTick,
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

        entry_point_binder_->bind("engine_service.ping", []() -> std::string {
            return "received a call to engine_service.ping";
        });

        entry_point_binder_->bind("engine_service.request_close", []() -> void {
            bool immediate_shutdown = false;
            FGenericPlatformMisc::RequestExit(immediate_shutdown);
        });

        entry_point_binder_->bind("engine_service.begin_tick", [this]() -> void {
            SP_ASSERT(frame_state_ == FrameState::Idle);

            // reset promises and futures
            frame_state_idle_promise_ = std::promise<void>();
            frame_state_executing_pre_tick_promise_ = std::promise<void>();
            frame_state_executing_post_tick_promise_ = std::promise<void>();

            frame_state_idle_future_ = frame_state_idle_promise_.get_future();
            frame_state_executing_pre_tick_future_ = frame_state_executing_pre_tick_promise_.get_future();
            frame_state_executing_post_tick_future_ = frame_state_executing_post_tick_promise_.get_future();

            // allow beginFrameHandler() to start executing, wait here until frame_state_ == FrameState::ExecutingPreTick
            frame_state_ = FrameState::RequestPreTick;
            frame_state_executing_pre_tick_future_.wait();
            SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        });

        entry_point_binder_->bind("engine_service.tick", [this]() -> void {
            SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);

            // allow beginFrameHandler() to finish executing, wait here until frame_state_ == FrameState::ExecutingPostTick
            work_queue_.reset();
            frame_state_executing_post_tick_future_.wait();
            SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        });

        entry_point_binder_->bind("engine_service.end_tick", [this]() -> void {
            SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);

            // allow endFrameHandler() to finish executing, wait here until frame_state_ == FrameState::Idle
            work_queue_.reset();
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

    void bindFuncNoUnreal(const std::string& service_name, const std::string& func_name, auto&& func)
    {
        entry_point_binder_->bind(
            service_name + "." + func_name,
            std::forward<decltype(func)>(func));
    }

    void bindFuncUnreal(const std::string& service_name, const std::string& func_name, auto&& func)
    {
        entry_point_binder_->bind(
            service_name + "." + func_name,
            WorkQueue::wrapFuncToExecuteInWorkQueueBlocking(work_queue_, std::forward<decltype(func)>(func)));
    }

    void close()
    {
        SP_ASSERT(frame_state_ == FrameState::Idle || frame_state_ == FrameState::RequestPreTick);

        // If frame_state_ == FrameState::RequestPreTick, then we know that engine_service.begin_tick()
        // has started executing but has not finished. In this case, we must set frame_state_ and
        // frame_state_executing_pre_tick_promise_, otherwise engine_service.begin_tick() will never
        // return, and this will prevent us from cleanly shutting down our RPC server.
                
        if (frame_state_ == FrameState::RequestPreTick) {
            frame_state_ = FrameState::ExecutingPreTick;
            frame_state_executing_pre_tick_promise_.set_value();
        }
    }

private:
    void beginFrameHandler()
    {
        if (frame_state_ == FrameState::RequestPreTick) {

            // allow begin_tick() to finish executing
            frame_state_ = FrameState::ExecutingPreTick;
            frame_state_executing_pre_tick_promise_.set_value();

            // execute all pre-tick work, wait here for tick() to unblock
            work_queue_.run();

            // update frame state
            frame_state_ = FrameState::ExecutingTick;
        }
    }

    void endFrameHandler()
    {
        if (frame_state_ == FrameState::ExecutingTick) {

            // allow tick() to finish executing
            frame_state_ = FrameState::ExecutingPostTick;
            frame_state_executing_post_tick_promise_.set_value();

            // execute all post-tick work, wait here for end_tick() to unblock
            work_queue_.run();

            // update frame state, allow end_tick() to finish executing
            frame_state_ = FrameState::Idle;
            frame_state_idle_promise_.set_value();
        }
    }

    TEntryPointBinder* entry_point_binder_ = nullptr;
    WorkQueue work_queue_;

    // FDelegateHandle objects corresponding to each event handler defined in this class
    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;

    // thread synchronization state
    std::atomic<FrameState> frame_state_;

    std::promise<void> frame_state_idle_promise_;
    std::promise<void> frame_state_executing_pre_tick_promise_;
    std::promise<void> frame_state_executing_post_tick_promise_;

    std::future<void> frame_state_idle_future_;
    std::future<void> frame_state_executing_pre_tick_future_;
    std::future<void> frame_state_executing_post_tick_future_;
};
