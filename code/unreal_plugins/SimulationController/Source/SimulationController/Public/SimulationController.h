#pragma once

#include <CoreMinimal.h>
#include <Modules/ModuleManager.h>

#include <memory>

class SimulationController : public IModuleInterface
{
public:

    // This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module.
    virtual void StartupModule() override;

    // This function may be called during shutdown to clean up your module. For modules that support dynamic reloading, we call this function before unloading the module.
    virtual void ShutdownModule() override;

    // Event handlers
    void postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources);
    void worldBeginPlayEventHandler();

private:

    void bindFunctionsToRpcServer();

    // DelegateHandles corresponding to each event handler defined in this class
    FDelegateHandle post_world_initialization_delegate_handle_;
    FDelegateHandle world_cleanup_delegate_handle_;
    FDelegateHandle world_begin_play_delegate_handle_;

    std::unique_ptr<class RpcServer> rpc_server_ = nullptr;
    UWorld* world_ = nullptr;
    bool is_world_begin_play_executed_ = false;
    std::unique_ptr<class AgentController> agent_controller_ = nullptr;
};
