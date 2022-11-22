#include "SimulationController.h"

#include <map>
#include <string>
#include <thread>
#include <vector>

#include <Engine/Engine.h>
#include <EngineUtils.h>
#include <GameFramework/GameModeBase.h>
#include <Kismet/GameplayStatics.h>
#include <Misc/CoreDelegates.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include "Agent.h"
#include "Assert/Assert.h"
#include "Box.h"
#include "CameraAgent.h"
#include "Config.h"
#include "ImitationLearningTask.h"
#include "NullTask.h"
#include "OpenBotAgent.h"
#include "PointGoalNavTask.h"
#include "RpcServer.h"
#include "SphereAgent.h"
#include "Task.h"
#include "Visualizer.h"

// Different possible frame states for thread synchronization
enum class FrameState : uint8_t
{
    Idle,
    RequestPreTick,
    ExecutingPreTick,
    ExecutingTick,
    ExecutingPostTick
};

void SimulationController::StartupModule()
{
    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("OpenBot")));
    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("SceneManager")));
    
    post_world_initialization_delegate_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SimulationController::postWorldInitializationEventHandler);
    world_cleanup_delegate_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &SimulationController::worldCleanupEventHandler);

    // required for adding thread synchronization logic
    begin_frame_delegate_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &SimulationController::beginFrameEventHandler);
    end_frame_delegate_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &SimulationController::endFrameEventHandler);
}

void SimulationController::ShutdownModule()
{
    // If this module is unloaded in the middle of simulation for some reason, raise an error because we do not support this and we want to know when this happens.
    // We expect worldCleanUpEvenHandler(...) to be called before ShutdownModule().
    ASSERT(!world_begin_play_delegate_handle_.IsValid());

    FCoreDelegates::OnEndFrame.Remove(end_frame_delegate_handle_);
    end_frame_delegate_handle_.Reset();

    FCoreDelegates::OnBeginFrame.Remove(begin_frame_delegate_handle_);
    begin_frame_delegate_handle_.Reset();

    FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_delegate_handle_);
    world_cleanup_delegate_handle_.Reset();
    
    FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_delegate_handle_);
    post_world_initialization_delegate_handle_.Reset();
}

void SimulationController::postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    ASSERT(world);

    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world) != nullptr) {
        
        auto world_path_name = Config::getValue<std::string>({"SIMULATION_CONTROLLER", "WORLD_PATH_NAME"});
        auto level_name = Config::getValue<std::string>({"SIMULATION_CONTROLLER", "LEVEL_NAME"});

        // if the current world is not the desired one, open the desired one
        if (world_path_name != "" && world_path_name != TCHAR_TO_UTF8(*(world->GetPathName()))) {

            // assert that we haven't already tried to open the level, because that means we failed
            ASSERT(!has_open_level_executed_);

            UGameplayStatics::OpenLevel(world, level_name.c_str());
            has_open_level_executed_ = true;

        } else {
            has_open_level_executed_ = false;

            // we expect worldCleanupEventHandler(...) to be called before a new world is created.
            ASSERT(!world_);

            // cache local reference to the UWorld
            world_ = world;

            // defer the rest of our initialization code until the OnWorldBeginPlay event
            world_begin_play_delegate_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &SimulationController::worldBeginPlayEventHandler);
        }
    }
}

void SimulationController::worldBeginPlayEventHandler()
{
    // Set console commands for syncing the game thread and RHI thread.
    // For more information on GTSyncType, see http://docs.unrealengine.com/en-US/SharingAndReleasing/LowLatencyFrameSyncing/index.html.
    GEngine->Exec(world_, TEXT("r.GTSyncType 1"));
    GEngine->Exec(world_, TEXT("r.OneFrameThreadLag 0"));

    // execute optional console commands from python client
    for (auto& command : Config::getValue<std::vector<std::string>>({"SIMULATION_CONTROLLER", "CUSTOM_UNREAL_CONSOLE_COMMANDS"})) {
        GEngine->Exec(world_, UTF8_TO_TCHAR(command.c_str()));
    }

    // Set fixed simulation step time in seconds. Check that the physics substepping parameters match our deired simulation step time.
    // See https://carla.readthedocs.io/en/latest/adv_synchrony_timestep
    UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
    ASSERT(physics_settings->bSubstepping);
    ASSERT(Config::getValue<float>({"SIMULATION_CONTROLLER", "SIMULATION_STEP_TIME_SECONDS"}) <= physics_settings->MaxSubstepDeltaTime * physics_settings->MaxSubsteps); 

    FApp::SetBenchmarking(true);
    FApp::SetFixedDeltaTime(Config::getValue<double>({"SIMULATION_CONTROLLER", "SIMULATION_STEP_TIME_SECONDS"}));

    // pause gameplay
    UGameplayStatics::SetGamePaused(world_, true);

    // create Agent
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "AGENT"}) == "CameraAgent") {
        agent_ = std::make_unique<CameraAgent>(world_);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "AGENT"}) == "OpenBotAgent") {
        agent_ = std::make_unique<OpenBotAgent>(world_);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "AGENT"}) == "SphereAgent") {
        agent_ = std::make_unique<SphereAgent>(world_);
    } else {
        ASSERT(false);
    }
    ASSERT(agent_);

    // create Task
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "TASK"}) == "ImitationLearningTask") {
        task_ = std::make_unique<ImitationLearningTask>(world_);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "TASK"}) == "NullTask") {
        task_ = std::make_unique<NullTask>();
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "TASK"}) == "PointGoalNavTask") {
        task_ = std::make_unique<PointGoalNavTask>(world_);
    } else {
        ASSERT(false);
    }
    ASSERT(task_);

    // create Visualizer
    visualizer_ = std::make_unique<Visualizer>(world_);
    ASSERT(visualizer_);

    // deferred initialization for Agent, Task, and Visualizer
    agent_->findObjectReferences(world_);
    task_->findObjectReferences(world_);
    visualizer_->findObjectReferences(world_);

    // initialize frame state used for thread synchronization
    frame_state_ = FrameState::Idle;

    // config values required for rpc communication
    auto hostname = Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IP"});
    auto port = Config::getValue<int>({"SIMULATION_CONTROLLER", "PORT"});

    rpc_server_ = std::make_unique<RpcServer>(hostname, port);
    ASSERT(rpc_server_);

    bindFunctionsToRpcServer();

    rpc_server_->launchWorkerThreads(1u);

    has_world_begin_play_executed_ = true;
}

void SimulationController::worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    ASSERT(world);

    // We only need to perform any additional steps if the world being cleaned up is the world we cached in our world_ member variable.
    if (world == world_) {

        // The worldCleanupEventHandler(...) function is called for all worlds, but some local state (such as rpc_server_ and agent_)
        // is initialized only when worldBeginPlayEventHandler(...) is called for a particular world. So we check if worldBeginPlayEventHandler(...)
        // has been executed.
        if(has_world_begin_play_executed_) {

            has_world_begin_play_executed_ = false;

            ASSERT(rpc_server_);
            rpc_server_->stop(); // stop the RPC server as we will no longer service client requests
            rpc_server_ = nullptr;

            ASSERT(visualizer_);
            visualizer_->cleanUpObjectReferences();
            visualizer_ = nullptr;

            ASSERT(task_);
            task_->cleanUpObjectReferences();
            task_ = nullptr;

            ASSERT(agent_);
            agent_->cleanUpObjectReferences();
            agent_ = nullptr;
        }

        // remove event handlers bound to this world before world gets cleaned up
        world_->OnWorldBeginPlay.Remove(world_begin_play_delegate_handle_);
        world_begin_play_delegate_handle_.Reset();

        // clear cached world_ pointer
        world_ = nullptr;
    }
}

void SimulationController::beginFrameEventHandler()
{
    // if beginTick(...) has indicated (via RequestPreTick framestate) that we should execute a frame of work
    if (frame_state_ == FrameState::RequestPreTick) {

        // update local state
        frame_state_ = FrameState::ExecutingPreTick;

        // unpause the game
        UGameplayStatics::SetGamePaused(world_, false);

        // execute all pre-tick sync work, wait here for tick(...) to reset work guard
        rpc_server_->runSync();

        // execute pre-tick work inside the task
        task_->beginFrame();

        // update local state
        frame_state_ = FrameState::ExecutingTick;
    }
}

void SimulationController::endFrameEventHandler()
{
    // if beginFrameEventHandler(...) has indicated that we are currently executing a frame of work
    if (frame_state_ == FrameState::ExecutingTick) {

        // update local state
        frame_state_ = FrameState::ExecutingPostTick;

        // execute post-tick work inside the task
        task_->endFrame();

        // allow tick() to finish executing
        end_frame_started_executing_promise_.set_value();

        // execute all post-tick sync work, wait here for endTick() to reset work guard
        rpc_server_->runSync();

        // pause the game
        UGameplayStatics::SetGamePaused(world_, true);

        // update local state
        frame_state_ = FrameState::Idle;

        // allow endTick(...) to finish executing
        end_frame_finished_executing_promise_.set_value();
    }
}

void SimulationController::bindFunctionsToRpcServer()
{
    rpc_server_->bindAsync("ping", []() -> std::string {
        return "SimulationController received a call to ping()...";
    });

    rpc_server_->bindAsync("close", []() -> void {
        constexpr auto immediate_shutdown = false;
        FGenericPlatformMisc::RequestExit(immediate_shutdown);
    });

    rpc_server_->bindAsync("getEndianness", []() -> std::string {
        uint32_t dummy = 0x01020304;
        return (reinterpret_cast<const char*>(&dummy)[3] == 1) ? "little" : "big";
    });

    rpc_server_->bindAsync("beginTick", [this]() -> void {
        ASSERT(frame_state_ == FrameState::Idle);

        // reinitialize end_frame_started_executing promise and future
        end_frame_started_executing_promise_ = std::promise<void>();
        end_frame_started_executing_future_ = end_frame_started_executing_promise_.get_future();

        // reinitialize end_frame_finished_executing promise and future
        end_frame_finished_executing_promise_ = std::promise<void>();
        end_frame_finished_executing_future_ = end_frame_finished_executing_promise_.get_future();

        // indicate that we want the game thread to execute one frame of work
        frame_state_ = FrameState::RequestPreTick;
    });

    rpc_server_->bindAsync("tick", [this]() -> void {
        ASSERT((frame_state_ == FrameState::ExecutingPreTick) || (frame_state_ == FrameState::RequestPreTick));

        // indicate that we want the game thread to stop blocking in beginFrame()
        rpc_server_->unblockRunSyncWhenFinishedExecuting();

        // wait here until the game thread has started executing endFrame()
        end_frame_started_executing_future_.wait();

        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
    });

    rpc_server_->bindAsync("endTick", [this]() -> void {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);

        // indicate that we want the game thread to stop blocking in endFrame()
        rpc_server_->unblockRunSyncWhenFinishedExecuting();

        // wait here until the game thread has finished executing endFrame()
        end_frame_finished_executing_future_.wait();

        ASSERT(frame_state_ == FrameState::Idle);
    });

    rpc_server_->bindAsync("getActionSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(agent_);
        return agent_->getActionSpace();
    });

    rpc_server_->bindAsync("getObservationSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(agent_);
        return agent_->getObservationSpace();
    });

    rpc_server_->bindAsync("getAgentStepInfoSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(agent_);
        return agent_->getStepInfoSpace();
    });

    rpc_server_->bindAsync("getTaskStepInfoSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(task_);
        return task_->getStepInfoSpace();
    });

    rpc_server_->bindSync("applyAction", [this](std::map<std::string, std::vector<float>> action) -> void {
        ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        ASSERT(agent_);
        agent_->applyAction(action);
    });

    rpc_server_->bindSync("getObservation", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(agent_);
        return agent_->getObservation();
    });

    rpc_server_->bindSync("getReward", [this]() -> float {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(task_);
        return task_->getReward();
    });

    rpc_server_->bindSync("isEpisodeDone", [this]() -> bool {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(task_);
        return task_->isEpisodeDone();
    });

    rpc_server_->bindSync("getAgentStepInfo", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(agent_);
        return agent_->getStepInfo();
    });

    rpc_server_->bindSync("getTaskStepInfo", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(task_);
        return task_->getStepInfo();
    });

    rpc_server_->bindSync("resetAgent", [this]() -> void {
        ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        ASSERT(agent_);
        agent_->reset();
    });

    rpc_server_->bindSync("resetTask", [this]() -> void {
        ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        ASSERT(task_);
        task_->reset();
    });

    rpc_server_->bindSync("isAgentReady", [this]() -> bool{
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(agent_);
        return agent_->isReady();
    });

    rpc_server_->bindSync("isTaskReady", [this]() -> bool{
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(task_);
        return task_->isReady();
    });
}

IMPLEMENT_MODULE(SimulationController, SimulationController)
