#pragma once

#include <CoreMinimal.h>
#include <Modules/ModuleManager.h>
#include <Templates/UniquePtr.h>

class SimulationController : public IModuleInterface
{
public:
    // IModuleInterface implementation overrides
    // This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
    virtual void StartupModule() override;

    // This function may be called during shutdown to clean up your module. For modules that support dynamic reloading, we call this function before unloading the module.
    virtual void ShutdownModule() override;

    // Event Handlers
    void postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources);
    void worldBeginPlayEventHandler();

private :
    // DelegateHandles corresponding to each Event Handler defined in this class
    FDelegateHandle post_world_initialization_delegate_handle_;
    FDelegateHandle world_cleanup_delegate_handle_;
    FDelegateHandle world_beginplay_delegate_handle_;

    // RPC server
    TUniquePtr<class RpcServer> rpc_server_ = nullptr;

    // local cache to game world
    UWorld* game_world_ = nullptr;

    void bindFunctionsToRpcServer();
};
