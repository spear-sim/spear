//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <atomic>
#include <concepts> // std::same_as
#include <future>   // std::promise, std::future

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Misc/CoreDelegates.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpEngine/WorkQueue.h"

template <typename TBasicEntryPointBinder>
concept CBasicEntryPointBinder = requires(TBasicEntryPointBinder basic_entry_point_binder) {
    { basic_entry_point_binder.bind("", []() -> void {}) } -> std::same_as<void>;
};

template <typename TEntryPointBinder>
concept CEntryPointBinder = requires(TEntryPointBinder entry_point_binder) {
    { entry_point_binder.bind("", "", []() -> void {}) } -> std::same_as<void>;
};

// Different possible frame states for thread synchronization
enum class FrameState
{
    Idle,
    RequestPreTick,
    ExecutingPreTick,
    ExecutingTick,
    ExecutingPostTick,
};

template <CBasicEntryPointBinder TBasicEntryPointBinder>
class EngineService {
public:
    EngineService(TBasicEntryPointBinder* basic_entry_point_binder)
    {
        basic_entry_point_binder_ = basic_entry_point_binder;

        begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &EngineService::beginFrameEventHandler);
        end_frame_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &EngineService::endFrameEventHandler);

        frame_state_ = FrameState::Idle;

        basic_entry_point_binder_->bind("engine_service.begin_tick", [this]() -> void {
            SP_LOG("0 - begin_tick");
            SP_ASSERT(frame_state_ == FrameState::Idle);
            // reset promises and futures
            frame_state_idle_promise_ = std::promise<void>();
            frame_state_executing_pre_tick_promise_ = std::promise<void>();
            frame_state_executing_post_tick_promise_ = std::promise<void>();

            frame_state_idle_future_ = frame_state_idle_promise_.get_future();
            frame_state_executing_pre_tick_future_ = frame_state_executing_pre_tick_promise_.get_future();
            frame_state_executing_post_tick_future_ = frame_state_executing_post_tick_promise_.get_future();

            // indicate that we want the game thread to advance the simulation, wait here until frame_state == FrameState::ExecutingPreTick
            frame_state_ = FrameState::RequestPreTick;
            frame_state_executing_pre_tick_future_.wait();
            SP_LOG("1 - begin_tick");
            SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        });

        basic_entry_point_binder_->bind("engine_service.tick", [this]() -> void {
            SP_LOG("0 - tick");
            SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
            // allow beginFrameEventHandler() to finish executing, wait here until frame_state == FrameState::ExecutingPostTick
            work_queue_.resetWorkGuard();
            frame_state_executing_post_tick_future_.wait();
            SP_LOG("2 - tick");

            SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        });

        basic_entry_point_binder_->bind("engine_service.end_tick", [this]() -> void {
            SP_LOG("0 - end_tick");
            SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
            // allow endFrameEventHandler() to finish executing, wait here until frame_state == FrameState::Idle
            work_queue_.resetWorkGuard();
            frame_state_idle_future_.wait();
            SP_LOG("2 - end_tick");

            SP_ASSERT(frame_state_ == FrameState::Idle);
        });

        basic_entry_point_binder_->bind("engine_service.ping", []() -> std::string {
            return "EngineService received a call to ping()...";
        });

        basic_entry_point_binder_->bind("engine_service.get_byte_order", []() -> std::string {
            uint32_t dummy = 0x01020304;
            return (reinterpret_cast<char*>(&dummy)[3] == 1) ? "little" : "big";
        });

        basic_entry_point_binder_->bind("engine_service.request_close", []() -> void {
            bool immediate_shutdown = false;
            FGenericPlatformMisc::RequestExit(immediate_shutdown);
        });
    }

    ~EngineService()
    {
        frame_state_ = FrameState::Idle;

        FCoreDelegates::OnEndFrame.Remove(end_frame_handle_);
        FCoreDelegates::OnBeginFrame.Remove(begin_frame_handle_);

        end_frame_handle_.Reset();
        begin_frame_handle_.Reset();
        
        basic_entry_point_binder_ = nullptr;
    }

    void bind(const std::string& service_name, const std::string& func_name, auto&& func)
    {
        basic_entry_point_binder_->bind(
            service_name + "." + func_name,
            WorkQueue::wrapFuncToExecuteInWorkQueueBlocking(work_queue_, std::forward<decltype(func)>(func))
        );

        //basic_entry_point_binder_->bind(
        //    service_name + ".async." + func_name,
        //    WorkQueue::wrapFuncToExecuteInWorkQueueNonBlocking(work_queue_, std::forward<decltype(func)>(func)));
    }

    void beginFrameEventHandler()
    {
        // if begin_tick() has indicated that we should advance the simulation
        if (frame_state_ == FrameState::RequestPreTick) {
            SP_LOG("0 - BeginFrameEventHandler");
            // update frame state, allow begin_tick() to finish executing
            frame_state_ = FrameState::ExecutingPreTick;
            frame_state_executing_pre_tick_promise_.set_value();

            // execute all pre-tick synchronous work, wait here for tick() to unblock us
            work_queue_.runIOContext();
            SP_LOG("1 - BeginFrameEventHandler");
            // update local state
            frame_state_ = FrameState::ExecutingTick;
        }
    }

    void endFrameEventHandler()
    {
        // if we are currently advancing the simulation
        if (frame_state_ == FrameState::ExecutingTick) {
            SP_LOG("0 - EndFrameEventHandler");
            // update frame state, allow tick() to finish executing
            frame_state_ = FrameState::ExecutingPostTick;
            frame_state_executing_post_tick_promise_.set_value();

            // execute all post-tick synchronous work, wait here for endTick() to unblock
            work_queue_.runIOContext();
            SP_LOG("1 - EndFrameEventHandler");

            // update frame state, allow endTick() to finish executing
            frame_state_ = FrameState::Idle;
            frame_state_idle_promise_.set_value();
        }
    }

private:
    TBasicEntryPointBinder* basic_entry_point_binder_ = nullptr;
    WorkQueue work_queue_;

    // FDelegateHandle objects corresponding to each event handler defined in this class
    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;

    // thread sychronization state
    std::atomic<FrameState> frame_state_;

    std::promise<void> frame_state_idle_promise_;
    std::promise<void> frame_state_executing_pre_tick_promise_;
    std::promise<void> frame_state_executing_post_tick_promise_;

    std::future<void> frame_state_idle_future_;
    std::future<void> frame_state_executing_pre_tick_future_;
    std::future<void> frame_state_executing_post_tick_future_;
};
