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

enum class FrameState : uint8_t;

class SimulationController : public IModuleInterface
{
public:

    // This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module.
    void StartupModule() override;

    // This function may be called during shutdown to clean up your module. For modules that support dynamic reloading, we call this function before unloading the module.
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
    FDelegateHandle begin_frame_delegate_handle_;
    FDelegateHandle end_frame_delegate_handle_;
    FDelegateHandle post_world_initialization_delegate_handle_;
    FDelegateHandle world_begin_play_delegate_handle_;
    FDelegateHandle world_cleanup_delegate_handle_;

    // store a local reference to a game world
    UWorld* world_ = nullptr;
    
    std::unique_ptr<Agent> agent_;
    std::unique_ptr<Task> task_;
    std::unique_ptr<Visualizer> visualizer_;
    std::unique_ptr<RpcServer> rpc_server_;

    bool has_world_begin_play_executed_ = false;
    bool has_open_level_executed_ = false;

    // thread sychronization
    std::atomic<FrameState> frame_state_;

    // used so tick() can wait until end_frame() has started executing before returning, reinitialized in begin_tick()
    std::promise<void> end_frame_started_executing_promise_;
    std::future<void> end_frame_started_executing_future_;

    // used so end_tick() can wait until end_frame() has finished executing before returning, reinitialized in begin_tick()
    std::promise<void> end_frame_finished_executing_promise_;
    std::future<void> end_frame_finished_executing_future_;
};
