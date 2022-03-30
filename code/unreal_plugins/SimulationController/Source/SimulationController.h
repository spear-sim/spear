#pragma once

#include <atomic>
#include <future>
#include <memory>

#include <CoreMinimal.h>
#include <Modules/ModuleManager.h>

// forward declarations
class RpcServer;
class AgentController;
enum class FrameState;

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
    void postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldBeginPlayEventHandler();
    void worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources);
    void beginFrameEventHandler();
    void endFrameEventHandler();
    void endFrameRTEventHandler();

    // DelegateHandles corresponding to each event handler defined in this class
    FDelegateHandle post_world_initialization_delegate_handle_;
    FDelegateHandle world_begin_play_delegate_handle_;
    FDelegateHandle world_cleanup_delegate_handle_;
    FDelegateHandle begin_frame_delegate_handle_;
    FDelegateHandle end_frame_delegate_handle_;
    FDelegateHandle end_frame_rt_delegate_handle_;

    // store a local reference to a game world
    UWorld* world_ = nullptr;
    
    std::unique_ptr<RpcServer> rpc_server_ = nullptr;
    std::unique_ptr<AgentController> agent_controller_ = nullptr;

    bool is_world_begin_play_executed_ = false;

    // thread sychronization elements
    std::atomic<FrameState> frame_state_;

    // used so tick() can wait until end_frame() has started executing before returning, reinitialized in begin_tick()
    std::promise<void> end_frame_started_executing_promise_;
    std::future<void> end_frame_started_executing_future_;

    // used so end_tick() can wait until end_frame() has finished executing before returning, reinitialized in begin_tick()
    std::promise<void> end_frame_finished_executing_promise_;
    std::future<void> end_frame_finished_executing_future_;
};
