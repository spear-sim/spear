// Copyright Epic Games, Inc. All Rights Reserved.

#include "SimulationController.h"

//remove
#include <iostream>
//this

#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Engine/Engine.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include "GameFramework/GameModeBase.h"
#include <Kismet/GameplayStatics.h>

#include "AgentController.h"
#include "Assert.h"
#include "Box.h"
#include "Config.h"
#include "RpcServer.h"
#include "SphereAgentController.h"

// different possible frame states for thread synchronization
enum class FrameState
{
    idle_,
    request_pre_tick_,
    executing_pre_tick_,
    executing_tick_,
    executing_post_tick_
};

void SimulationController::StartupModule()
{
    // Initialize config system
    Config::initialize();

    // Required to add ActorSpawnedEventHandler
    post_world_initialization_delegate_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SimulationController::postWorldInitializationEventHandler);

    // Required to reset any custom logic during a world cleanup
    world_cleanup_delegate_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &SimulationController::worldCleanupEventHandler);

    begin_frame_delegate_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &SimulationController::beginFrameEventHandler);
    end_frame_delegate_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &SimulationController::endFrameEventHandler);
    end_frame_rt_delegate_handle_ = FCoreDelegates::OnEndFrameRT.AddRaw(this, &SimulationController::endFrameRTEventHandler);

    end_frame_started_executing_promise_ = std::promise<bool>();
    end_frame_started_executing_future_ = end_frame_started_executing_promise_.get_future();

    end_frame_finished_executing_promise_ = std::promise<bool>();
    end_frame_finished_executing_future_ = end_frame_finished_executing_promise_.get_future();
    
    frame_state_.store(FrameState::idle_, std::memory_order_seq_cst);
}

void SimulationController::ShutdownModule()
{
    // If this module is unloaded in the middle of simulation for some reason, raise an error because we do not support this and we want to know when this happens.
    // We expect worldCleanUpEvenHandler() to be called before ShutdownModule(). 
    ASSERT(!world_begin_play_delegate_handle_.IsValid());

    // Remove event handlers used by this module
    FCoreDelegates::OnEndFrameRT.Remove(end_frame_rt_delegate_handle_);
    end_frame_rt_delegate_handle_.Reset();

    FCoreDelegates::OnEndFrame.Remove(end_frame_delegate_handle_);
    end_frame_delegate_handle_.Reset();

    FCoreDelegates::OnBeginFrame.Remove(begin_frame_delegate_handle_);
    begin_frame_delegate_handle_.Reset();

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

void SimulationController::worldBeginPlayEventHandler()
{
    // Set few console commands for syncing Game Thread (GT) and RHI thread.
    // For more information on GTSyncType, see http://docs.unrealengine.com/en-US/SharingAndReleasing/LowLatencyFrameSyncing/index.html.
    GEngine->Exec(world_, TEXT("r.GTSyncType 1"));
    GEngine->Exec(world_, TEXT("r.OneFrameThreadLag 0"));

    // Set fixed simulation step time in seconds
    FApp::SetBenchmarking(true);
    FApp::SetFixedDeltaTime(Config::getValue<double>({"INTERIORSIM", "SIMULATION_STEP_TIME_SECONDS"}));

    // // Read and set random seed value
    // Seed = Config::getValue<int>({"INTERIORSIM", "RANDOM_SEED"});
    // SetRandomStreamSeed(Seed); // @TODO: complete this

    UGameplayStatics::SetGamePaused(world_, true);

    // @TODO: Read config to decide which type of AgentController to create
    if(Config::getValue<std::string>({"INTERIORSIM", "AGENT_NAME"}) == "SphereAgent") {
        agent_controller_ = std::make_unique<SphereAgentController>(world_); // passing around UWorld pointer can be dangerous!
    }
    else {
        ASSERT(false);
    }

    // Add separate Task class (compute reward)

    
    // config values required for rpc communication
    std::string hostname = Config::getValue<std::string>({"INTERIORSIM", "IP"});
    int port = Config::getValue<int>({"INTERIORSIM", "PORT"});

    // First argument is false because we want to start the server is asynchronous mode, otherwise it will block the gamethread from executing anything!
    rpc_server_ = std::make_unique<RpcServer>(hostname, port);
    ASSERT(rpc_server_);
    bindFunctionsToRpcServer();
    rpc_server_->launchWorkerThreads(Config::getValue<int>({"INTERIORSIM", "RPC_WORKER_THREADS"}));

    // set this boolean to true since worldbeginplay executed
    is_world_begin_play_executed_ = true;
}

void SimulationController::worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    ASSERT(world);

    if (world->IsGameWorld()) {
        ASSERT(world_);

        // OnWorldCleanUp is called for all worlds. rpc_server_ and agent_controller_ is created only when a Game World's begin play event is launched.
        if(is_world_begin_play_executed_) {
            ASSERT(rpc_server_);
            rpc_server_->stop(); // stop the RPC server as we will no longer service client requests

            agent_controller_.reset(nullptr);
        }

        // Remove event handlers bound to this world before world gets cleaned up
        world->OnWorldBeginPlay.Remove(world_begin_play_delegate_handle_);
        world_begin_play_delegate_handle_.Reset();

        // Clear local cache
        world_ = nullptr;
    }
}

void SimulationController::beginFrameEventHandler()
{   
    // std::cout << "--------------------------------------" << std::endl;
    // std::cout << "---BeginFrame---Executing beginFrameEventHandler() " << std::endl;

    // if server_func_begin_tick() has indicated that we should execute a frame of work
    if (frame_state_.load(std::memory_order_seq_cst) == FrameState::request_pre_tick_) {

        std::cout << "c++: OnBeginFrame: GFrameCounter " << GFrameCounter << std::endl;
        std::cout << "c++: OnBeginFrame: GFrameNumber " << GFrameNumber << std::endl;
        std::cout << "c++: OnBeginFrame: GFrameNumberRenderThread " << GFrameNumberRenderThread << std::endl;

        // update frame state
        frame_state_.store(FrameState::executing_pre_tick_, std::memory_order_seq_cst);

        std::cout << "c++: BeginFrame: Before game is unpaused" << std::endl;

        // unpause the game
        UGameplayStatics::SetGamePaused(world_, false);

        std::cout << "c++: BeginFrame: runSync() block" << std::endl;

        // execute all pre-tick sync work, wait here for tick() to reset work guard
        rpc_server_->RunSync();

        std::cout << "c++: BeginFrame: runSync() is unblocked" << std::endl;
    
        // reinitialze the io_context and work guard after runSync
        rpc_server_->reinitializeIOContextAndWorkGuard();

        // update frame state
        frame_state_.store(FrameState::executing_tick_, std::memory_order_seq_cst);
    }
    // std::cout << "----------------------------------------" << std::endl;
}

void SimulationController::endFrameEventHandler()
{
    // std::cout << "-----------------------------------" << std::endl;
    // std::cout << "---EndFrame---Executing endFrameEventHandler() " << std::endl;

    if (frame_state_.load(std::memory_order_seq_cst) == FrameState::executing_tick_) {

        std::cout << "c++: EndFrame: started..." << std::endl;

        std::cout << "c++: EndFrame: set end_frame_started_executing_promise_" << std::endl;
        // allow tick() to finish executing
        end_frame_started_executing_promise_.set_value(true);

        // update frame state
        frame_state_.store(FrameState::executing_post_tick_, std::memory_order_seq_cst);

        std::cout << "c++: EndFrame: Before game is paused" << std::endl;

        // pause the game
        UGameplayStatics::SetGamePaused(world_, true);

        std::cout << "c++: EndFrame: runSync() block" << std::endl;

        // execute all post-tick sync work, wait here for server_func_end_tick() to reset work guard
        rpc_server_->RunSync();

        std::cout << "c++: EndFrame: runSync() is unblocked" << std::endl;

        // reinitialze the io_context and work guard after runSync
        rpc_server_->reinitializeIOContextAndWorkGuard();

        // update frame state
        frame_state_.store(FrameState::idle_, std::memory_order_seq_cst);

        std::cout << "c++: EndFrame: set end_frame_finished_executing_promise_ and end endframe" << std::endl;
        // notify that end frame has finished executing
        end_frame_finished_executing_promise_.set_value(true);

        std::cout << "c++: OnEndFrame: GFrameCounter " << GFrameCounter << std::endl;
        std::cout << "c++: OnEndFrame: GFrameNumber " << GFrameNumber << std::endl;
        std::cout << "c++: OnEndFrame: GFrameNumberRenderThread " << GFrameNumberRenderThread << std::endl;
    }

    // std::cout << "---------------------------------------" << std::endl;
}

void SimulationController::endFrameRTEventHandler()
{
    // std::cout << "c++: OnEndFrameRT: GFrameCounter " << GFrameCounter << std::endl;
    // std::cout << "c++: OnEndFrameRT: GFrameNumber " << GFrameNumber << std::endl;
    // std::cout << "c++: OnEndFrameRT: GFrameNumberRenderThread " << GFrameNumberRenderThread << std::endl;
}

void SimulationController::bindFunctionsToRpcServer()
{
    rpc_server_->bindAsync("ping", []() -> std::string {
        return "received ping";
    });

    rpc_server_->bindAsync("close", []() -> void {
        constexpr bool immediate_shutdown = false;
        FGenericPlatformMisc::RequestExit(immediate_shutdown);
    });

    rpc_server_->bindAsync("pause", [this]() -> bool {
        return UGameplayStatics::SetGamePaused(world_, true);
    });

    rpc_server_->bindAsync("unPause", [this]() -> bool {
        return UGameplayStatics::SetGamePaused(world_, false);
    });

    rpc_server_->bindAsync("isPaused", [this]() -> bool {
        return world_->GetAuthGameMode()->IsPaused();
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

    rpc_server_->bindAsync("beginTick", [this]() -> void {
        
        std::cout << "c++: beginTick----start--------------" << std::endl;

        ASSERT(frame_state_.load(std::memory_order_seq_cst) == FrameState::idle_);

        // reinitialize end_frame_started_executing promise and future
        end_frame_started_executing_promise_ = std::promise<bool>();
        end_frame_started_executing_future_ = end_frame_started_executing_promise_.get_future();

        // reinitialize end_frame_finished_executing promise and future
        end_frame_finished_executing_promise_ = std::promise<bool>();
        end_frame_finished_executing_future_ = end_frame_finished_executing_promise_.get_future();

        // indicate that we want the game thread to execute one frame of work
        frame_state_.store(FrameState::request_pre_tick_, std::memory_order_seq_cst);

        std::cout << "c++: beginTick----end--------------" << std::endl;
    });

    rpc_server_->bindAsync("tick", [this]() -> void {
        
        std::cout << "c++: tick----start--------------" << std::endl;

        ASSERT((frame_state_.load(std::memory_order_seq_cst) == FrameState::executing_pre_tick_) || (frame_state_.load(std::memory_order_seq_cst) == FrameState::request_pre_tick_));

        // indicate that we want the game thread to stop blocking in begin_frame()
        rpc_server_->resetWorkGuard();

        // wait here until the game thread has started executing end_frame()
        std::cout << "c++: tick---- resetted workguard and now waiting for end_frame_started_executing_promise_ to set value" << std::endl;
        end_frame_started_executing_future_.wait();
        std::cout << "c++: tick----end_frame_started_executing_promise_ has set value" << std::endl;

        ASSERT(frame_state_.load(std::memory_order_seq_cst) == FrameState::executing_post_tick_);

        std::cout << "c++: tick----end--------------" << std::endl;
    });

    rpc_server_->bindAsync("endTick", [this]() -> void {
        
        std::cout << "c++: endTick----start----------------" << std::endl;

        ASSERT(frame_state_.load(std::memory_order_seq_cst) == FrameState::executing_post_tick_);

        // indicate that we want the game thread to stop blocking in end_frame()
        rpc_server_->resetWorkGuard();

        std::cout << "c++: endtick---- resetted workguard and now waiting for end_frame_finished_executing_promise_ to set value" << std::endl;
        // wait here until the game thread has finished executing end_frame()
        end_frame_finished_executing_future_.wait();
        std::cout << "c++: endtick----end_frame_finished_executing_promise_ has set value" << std::endl;

        std::cout << "c++: endTick----end----------------" << std::endl;
    });

    rpc_server_->bindAsync("getObservationSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(agent_controller_);
        return agent_controller_->getObservationSpace();
    });

    rpc_server_->bindAsync("getActionSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(agent_controller_);
        return agent_controller_->getActionSpace();
    });

    rpc_server_->bindSync("getObservation", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        ASSERT(agent_controller_);
        return agent_controller_->getObservation();
    });

    rpc_server_->bindSync("applyAction", [this](std::map<std::string, std::vector<float>> action) -> void {
        ASSERT(agent_controller_);
        agent_controller_->applyAction(action);
    });
}

IMPLEMENT_MODULE(SimulationController, SimulationController)
