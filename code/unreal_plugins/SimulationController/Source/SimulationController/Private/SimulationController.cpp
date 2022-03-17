// Copyright Epic Games, Inc. All Rights Reserved.

#include "SimulationController.h"

#include <Engine/Engine.h>
#include <Engine/World.h>
#include <EngineUtils.h>

//#include <SimpleVehicle/SimModeSimpleVehicle.h>
//#include <UrdfBot/SimModeUrdfBot.h>

#include "Assert.h"
#include "Config.h"
#include "RpcServer.h"

#define LOCTEXT_NAMESPACE "SimulationController"

void SimulationController::StartupModule()
{
    // Initialize config system
    Config::initialize();

    // Read config values required for rpc communication
    FString hostname = Config::getValue<std::string>({"UNREALAI", "IP"}).c_str();
    int port = Config::getValue<int>({"UNREALAI", "PORT"});

    // Create and launch rpc server
    // First argument is false because we want to start the server is asynchronous mode, otherwise it will block the gamethread from executing anything!
    // The client will call a function that will take care of setting synchronous mode.
    rpc_server_ = MakeUnique<RpcServer>(false, TCHAR_TO_UTF8(*hostname), port);  // TCHAR_TO_UTF8 for FString -> std::string
    ASSERT(rpc_server_);
    rpc_server_->asyncRun(std::max(std::thread::hardware_concurrency(), 4u) - 2u); // launch worker threads
    bindFunctionsToRpcServer();

    // Required to add ActorSpawnedEventHandler
    post_world_initialization_delegate_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SimulationController::postWorldInitializationEventHandler);

    // Required to reset any custom logic during a world cleanup
    world_cleanup_delegate_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &SimulationController::worldCleanupEventHandler);

    // OnBeginFrameDelegate = FCoreDelegates::OnBeginFrame.AddRaw(this, &UnrealRLManager::OnBeginFrame);
}

void SimulationController::ShutdownModule()
{
    // If this module is unloaded in the middle of simulation for some reason, raise an error because we do not support this and we want to know when this happens
    ASSERT(!world_beginplay_delegate_handle_.IsValid());
    ASSERT(rpc_server_);

    // Remove event handlers used by this module
    FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_delegate_handle_);
    world_cleanup_delegate_handle_.Reset();
    
    FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_delegate_handle_);
    post_world_initialization_delegate_handle_.Reset();

    // Quit worker threads and stop the RPC server
    rpc_server_->stop();

    // Terminate config system as we will not use it anymore
    Config::terminate();
}

void SimulationController::postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    ASSERT(world);

    if (world->IsGameWorld())
    {
        // Check if game_world_ is valid, and if it is, we do not support mulitple Game worlds and we need to know about this. This should happen only once in program lifecycle.
        ASSERT(!game_world_);

        // Cache local reference of World instance as this is required in other parts of this class.
        game_world_ = world;
        
        // Required to assign an AgentController based on config param
        world_beginplay_delegate_handle_ = world->OnWorldBeginPlay.AddRaw(this, &SimulationController::worldBeginPlayEventHandler);
    }
}

void SimulationController::worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    ASSERT(world);

    if (world->IsGameWorld())
    {
        // Remove event handlers bound to this world before world gets cleaned up
        world->OnWorldBeginPlay.Remove(world_beginplay_delegate_handle_);
        world_beginplay_delegate_handle_.Reset();

        // Clear local cache
        game_world_ = nullptr;
    }
}

void SimulationController::worldBeginPlayEventHandler()
{
    UE_LOG(LogTemp, Warning, TEXT(" "));
    UE_LOG(LogTemp, Warning, TEXT("OnWorldBeginPlay called...."));
    UE_LOG(LogTemp, Warning, TEXT(" "));

    // Set few console commands for syncing Game Thread (GT) and RHI thread.
    // For more information on GTSyncType, see http://docs.unrealengine.com/en-US/SharingAndReleasing/LowLatencyFrameSyncing/index.html.
    GEngine->Exec(game_world_, TEXT("r.GTSyncType 1"));
    GEngine->Exec(game_world_, TEXT("r.OneFrameThreadLag 0"));

    // Set fixed simulation step time in seconds
    // FApp::SetBenchmarking(true);
    // FApp::SetFixedDeltaTime(unrealrl::Config::getValue<double>({"UNREALAI", "SIMULATION_STEP_TIME_SECONDS"}));

    // // Read and set random seed value
    // Seed = unrealrl::Config::getValue<int>({"UNREALAI", "RANDOM_SEED"});
    // SetRandomStreamSeed(Seed);

    // @TODO: Read config to decide which AgentController to create

}

void SimulationController::bindFunctionsToRpcServer()
{
    rpc_server_->bindAsync("Ping", []() -> std::string {
        return "received ping";
    });

    rpc_server_->bindAsync("Echo", [](std::string echo_string) -> std::string {
        return std::string("echoing -> ") + echo_string;
    });

    // rpc_server_->bindSync("")
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(SimulationController, SimulationController)
