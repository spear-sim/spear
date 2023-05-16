//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <atomic>
#include <future>
#include <memory>

#include <CoreMinimal.h>
#include <Engine/World.h>
#include <Modules/ModuleManager.h>

class Agent;
class RpcServer;
class Task;
class Visualizer;
enum class FrameState;

class SimulationController : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

private:
    // bind functions to rpc server to service client requests
    void bindFunctionsToRpcServer();

    // event handlers
    void beginFrameEventHandler();
    void endFrameEventHandler();
    void postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldBeginPlayEventHandler();
    void worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources);

    // FDelegateHandle objects corresponding to each event handler defined in this class
    FDelegateHandle begin_frame_handle_;
    FDelegateHandle end_frame_handle_;
    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_begin_play_handle_;
    FDelegateHandle world_cleanup_handle_;

    // store a local reference to a game world
    UWorld* world_ = nullptr;
    
    std::unique_ptr<Agent> agent_;
    std::unique_ptr<Task> task_;
    std::unique_ptr<Visualizer> visualizer_;
    std::unique_ptr<RpcServer> rpc_server_;

    bool has_world_begin_play_executed_ = false;
    bool open_level_pending_ = false;

    // thread sychronization
    std::atomic<FrameState> frame_state_;

    std::promise<void> frame_state_idle_promise_;
    std::promise<void> frame_state_executing_pre_tick_promise_;
    std::promise<void> frame_state_executing_post_tick_promise_;

    std::future<void> frame_state_idle_future_;
    std::future<void> frame_state_executing_pre_tick_future_;
    std::future<void> frame_state_executing_post_tick_future_;
};
