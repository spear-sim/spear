// Copyright Epic Games, Inc. All Rights Reserved.

#include "SimulationController.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Engine/Engine.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <Kismet/GameplayStatics.h>

//#include <SimpleVehicle/SimModeSimpleVehicle.h>
//#include <UrdfBot/SimModeUrdfBot.h>

#include "AgentController.h"
#include "Assert.h"
#include "Box.h"
#include "Config.h"
#include "RpcServer.h"
#include "SphereAgentController.h"

#define LOCTEXT_NAMESPACE "SimulationController"

void SimulationController::StartupModule()
{
    // Initialize config system
    Config::initialize();

    // Required to add ActorSpawnedEventHandler
    post_world_initialization_delegate_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SimulationController::postWorldInitializationEventHandler);

    // Required to reset any custom logic during a world cleanup
    world_cleanup_delegate_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &SimulationController::worldCleanupEventHandler);

    // OnBeginFrameDelegate = FCoreDelegates::OnBeginFrame.AddRaw(this, &SimulationController::OnBeginFrame);
    // OnEndFrameDelegate = FCoreDelegates::OnEndFrame.AddRaw(this, &SimulationController::OnEndFrame);
}

void SimulationController::ShutdownModule()
{
    // If this module is unloaded in the middle of simulation for some reason, raise an error because we do not support this and we want to know when this happens.
    // We expect worldCleanUpEvenHandler() to be called before ShutdownModule(). 
    ASSERT(!world_begin_play_delegate_handle_.IsValid());

    // Remove event handlers used by this module
    FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_delegate_handle_);
    world_cleanup_delegate_handle_.Reset();
    
    FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_delegate_handle_);
    post_world_initialization_delegate_handle_.Reset();

    // Terminate config system as we will not use it anymore
    Config::terminate();
}

void SimulationController::postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    ASSERT(world);

    if (world->IsGameWorld()) {
        // Check if world_ is valid, and if it is, we do not support mulitple Game worlds and we need to know about this. There should only be one Game World..
        ASSERT(!world_);

        // Cache local reference of World instance as this is required in other parts of this class.
        world_ = world;
        
        // Required to assign an AgentController based on config param
        world_begin_play_delegate_handle_ = world->OnWorldBeginPlay.AddRaw(this, &SimulationController::worldBeginPlayEventHandler);
    }
}

void SimulationController::worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    ASSERT(world);

    if (world->IsGameWorld()) {
        ASSERT(world_);

        // OnWorldCleanUp is called for all worlds. rpc_server_ is created only when a Game World's begin play event is launched.
        if(is_world_begin_play_executed_) {
            ASSERT(rpc_server_);
            rpc_server_->stop(); // stop the RPC server
        }

        // Remove event handlers bound to this world before world gets cleaned up
        world->OnWorldBeginPlay.Remove(world_begin_play_delegate_handle_);
        world_begin_play_delegate_handle_.Reset();

        // Clear local cache
        world_ = nullptr;
    }
}

void SimulationController::worldBeginPlayEventHandler()
{
    // Set few console commands for syncing Game Thread (GT) and RHI thread.
    // For more information on GTSyncType, see http://docs.unrealengine.com/en-US/SharingAndReleasing/LowLatencyFrameSyncing/index.html.
    GEngine->Exec(world_, TEXT("r.GTSyncType 1"));
    GEngine->Exec(world_, TEXT("r.OneFrameThreadLag 0"));

    // Set fixed simulation step time in seconds
    // FApp::SetBenchmarking(true);
    // FApp::SetFixedDeltaTime(unrealrl::Config::getValue<double>({"INTERIORSIM", "SIMULATION_STEP_TIME_SECONDS"}));

    // // Read and set random seed value
    // Seed = unrealrl::Config::getValue<int>({"INTERIORSIM", "RANDOM_SEED"});
    // SetRandomStreamSeed(Seed); // @TODO: complete this

    // @TODO: Read config to decide which AgentController to create
    if(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "AGENT_NAME"}) == "SphereAgent") {
        agent_controller_ = std::make_unique<SphereAgentController>(world_); // passing around UWorld pointer can be dangerous!
    }

    // Add separate Task class (compute reward)

    
    // config values required for rpc communication
    constexpr bool is_sync = false;
    std::string hostname = Config::getValue<std::string>({"INTERIORSIM", "IP"});
    int port = Config::getValue<int>({"INTERIORSIM", "PORT"});

    // First argument is false because we want to start the server is asynchronous mode, otherwise it will block the gamethread from executing anything!
    rpc_server_ = std::make_unique<RpcServer>(is_sync, hostname, port);
    ASSERT(rpc_server_);
    bindFunctionsToRpcServer();
    rpc_server_->asyncRun(Config::getValue<int>({"INTERIORSIM", "RPC_WORKER_THREADS"})); // launch worker threads


    // set this boolean to true since worldbeginplay executed
    is_world_begin_play_executed_ = true;
}

void SimulationController::bindFunctionsToRpcServer()
{
    rpc_server_->bindAsync("ping", []() -> std::string {
        return "received ping";
    });

    rpc_server_->bindAsync("echo", [](std::string echo_string) -> std::string {
        return std::string("echoing -> ") + echo_string;
    });

    rpc_server_->bindAsync("close", []() -> void {
        FGenericPlatformMisc::RequestExit(false);
    });

    rpc_server_->bindAsync("pause", [this]() -> bool {
        return UGameplayStatics::SetGamePaused(this->world_, true);
    });

    rpc_server_->bindAsync("unPause", [this]() -> bool {
        return UGameplayStatics::SetGamePaused(this->world_, false);
    });

    rpc_server_->bindAsync("getEndianness", []() -> uint8_t {
        enum class Endianness : uint8_t
        {
            LittleEndian = 0,
            BigEndian = 1,
        };        
        uint32_t Num = 0x01020304;
        return (reinterpret_cast<const char*>(&Num)[3] == 1) ? static_cast<uint8_t>(Endianness::LittleEndian) : static_cast<uint8_t>(Endianness::BigEndian);
    });

    rpc_server_->bindAsync("getObservationSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(this->agent_controller_);
        return this->agent_controller_.get()->getObservationSpace();
    });

    rpc_server_->bindAsync("getActionSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(this->agent_controller_);
        return this->agent_controller_.get()->getActionSpace();
    });

    rpc_server_->bindAsync("getObservation", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        ASSERT(this->agent_controller_);
        return this->agent_controller_.get()->getObservation();
    });

    rpc_server_->bindAsync("applyAction", [this](std::map<std::string, std::vector<float>> action) -> void {
        ASSERT(this->agent_controller_);
        this->agent_controller_.get()->applyAction(action);
    });
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(SimulationController, SimulationController)
