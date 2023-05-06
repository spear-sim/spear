//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/SimulationController.h"

#include <atomic>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Engine/Engine.h>
#include <Engine/World.h>
#include <GameFramework/GameModeBase.h>
#include <Kismet/GameplayStatics.h>
#include <Misc/CoreDelegates.h>
#include <Modules/ModuleManager.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/Agent.h"
#include "SimulationController/CameraAgent.h"
#include "SimulationController/ImitationLearningTask.h"
#include "SimulationController/NullTask.h"
#include "SimulationController/OpenBotAgent.h"
#include "SimulationController/PointGoalNavTask.h"
#include "SimulationController/RpcServer.h"
#include "SimulationController/SphereAgent.h"
#include "SimulationController/Task.h"
#include "SimulationController/UrdfBotAgent.h"
#include "SimulationController/Visualizer.h"

// Different possible frame states for thread synchronization
enum class FrameState
{
    Idle,
    RequestPreTick,
    ExecutingPreTick,
    ExecutingTick,
    ExecutingPostTick,
};

void SimulationController::StartupModule()
{
    std::cout << "[SPEAR | SimulationController.cpp] SimulationController::StartupModule" << std::endl;

    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));

    if (!Config::s_initialized_) {
        return;
    }

    post_world_initialization_delegate_handle_ =
        FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SimulationController::postWorldInitializationEventHandler);
    world_cleanup_delegate_handle_ =
        FWorldDelegates::OnWorldCleanup.AddRaw(this, &SimulationController::worldCleanupEventHandler);

    begin_frame_delegate_handle_ =
        FCoreDelegates::OnBeginFrame.AddRaw(this, &SimulationController::beginFrameEventHandler);
    end_frame_delegate_handle_ =
        FCoreDelegates::OnEndFrame.AddRaw(this, &SimulationController::endFrameEventHandler);
}

void SimulationController::ShutdownModule()
{
    std::cout << "[SPEAR | SimulationController.cpp] SimulationController::ShutdownModule" << std::endl;

    if (!Config::s_initialized_) {
        return;
    }

    FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_delegate_handle_);
    world_cleanup_delegate_handle_.Reset();

    FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_delegate_handle_);
    post_world_initialization_delegate_handle_.Reset();

    // If this module is unloaded in the middle of simulation for some reason, raise an error.
    // We expect worldCleanUpEvenHandler(...) to be called before ShutdownModule().
    ASSERT(!world_begin_play_delegate_handle_.IsValid());

    FCoreDelegates::OnEndFrame.Remove(end_frame_delegate_handle_);
    end_frame_delegate_handle_.Reset();

    FCoreDelegates::OnBeginFrame.Remove(begin_frame_delegate_handle_);
    begin_frame_delegate_handle_.Reset();
}

void SimulationController::postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    std::cout << "[SPEAR | SimulationController.cpp] SimulationController::postWorldInitializationEventHandler" << std::endl;

    ASSERT(world);

    #if WITH_EDITOR
        bool world_is_ready = world->IsGameWorld();
    #else
        bool world_is_ready = world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world);
    #endif

    if (world_is_ready) {

        std::string scene_id = Config::get<std::string>("SIMULATION_CONTROLLER.SCENE_ID");
        std::string map_id = Config::get<std::string>("SIMULATION_CONTROLLER.MAP_ID");

        std::string desired_world_path_name;
        std::string desired_level_name;
        if (scene_id != "") {
            if (map_id == "") {
                map_id = scene_id;
            }
            desired_world_path_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id + "." + map_id;
            desired_level_name      = "/Game/Scenes/" + scene_id + "/Maps/" + map_id;            
        }

        // if the current world is not the desired one, open the desired one
        bool open_level = desired_world_path_name != "" && desired_world_path_name != Unreal::toStdString(world->GetPathName());

        std::cout << "[SPEAR | SimulationController.cpp] scene_id:                " << scene_id << std::endl;
        std::cout << "[SPEAR | SimulationController.cpp] map_id:                  " << map_id << std::endl;
        std::cout << "[SPEAR | SimulationController.cpp] desired_world_path_name: " << desired_world_path_name << std::endl;
        std::cout << "[SPEAR | SimulationController.cpp] desired_level_name:      " << desired_level_name << std::endl;
        std::cout << "[SPEAR | SimulationController.cpp] world->GetPathName():    " << Unreal::toStdString(world->GetPathName()) << std::endl;
        std::cout << "[SPEAR | SimulationController.cpp] open_level:              " << open_level << std::endl;

        if (open_level) {
            std::cout << "[SPEAR | SimulationController.cpp] Opening level: " << desired_level_name << std::endl;

            // if we're at this line of code and OpenLevel is already pending, it means we failed
            ASSERT(!open_level_is_pending_);

            UGameplayStatics::OpenLevel(world, Unreal::toFName(desired_level_name));
            open_level_is_pending_ = true;

        } else {
            open_level_is_pending_ = false;

            // we expect worldCleanupEventHandler(...) to be called before a new world is created
            ASSERT(!world_);

            // cache local reference to the UWorld
            world_ = world;

            // We need to defer initializing this handler until after we have a valid world_ pointer,
            // and we defer the rest of our initialization code until the OnWorldBeginPlay event. We
            // wrap this code in an if block to enable interactive navigation mode, which will potentially
            // need to load a new map via the config system, but should not initialize the rest of the
            // code.
            if (Config::get<std::string>("SIMULATION_CONTROLLER.INTERACTION_MODE") == "programmatic") {
                world_begin_play_delegate_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &SimulationController::worldBeginPlayEventHandler);
            }
        }
    }
}

void SimulationController::worldBeginPlayEventHandler()
{
    std::cout << "[SPEAR | SimulationController.cpp] SimulationController::worldBeginPlayEventHandler" << std::endl;

    // execute optional console commands from python client
    for (auto& command : Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CUSTOM_UNREAL_CONSOLE_COMMANDS")) {
        GEngine->Exec(world_, *Unreal::toFString(command));
    }

    // set physics parameters
    UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
    physics_settings->bEnableEnhancedDeterminism = Config::get<bool>("SIMULATION_CONTROLLER.PHYSICS.ENABLE_ENHANCED_DETERMINISM");
    physics_settings->bSubstepping = Config::get<bool>("SIMULATION_CONTROLLER.PHYSICS.ENABLE_SUBSTEPPING");
    physics_settings->MaxSubstepDeltaTime = Config::get<float>("SIMULATION_CONTROLLER.PHYSICS.MAX_SUBSTEP_DELTA_TIME");
    physics_settings->MaxSubsteps = Config::get<int32>("SIMULATION_CONTROLLER.PHYSICS.MAX_SUBSTEPS");
    physics_settings->ContactOffsetMultiplier = Config::get<float>("SIMULATION_CONTROLLER.PHYSICS.CONTACT_OFFSET_MULTIPLIER");
    physics_settings->MinContactOffset = Config::get<float>("SIMULATION_CONTROLLER.PHYSICS.MIN_CONTACT_OFFSET");
    physics_settings->MaxContactOffset = Config::get<float>("SIMULATION_CONTROLLER.PHYSICS.MAX_CONTACT_OFFSET");

    // Check that the physics substepping parameters match our deired simulation step time.
    // See https://carla.readthedocs.io/en/latest/adv_synchrony_timestep for more details.
    if (physics_settings->bSubstepping) {
        ASSERT(Config::get<float>("SIMULATION_CONTROLLER.PHYSICS.SIMULATION_STEP_TIME_SECONDS") <= physics_settings->MaxSubstepDeltaTime * physics_settings->MaxSubsteps);
    }

    // set fixed simulation step time in seconds
    FApp::SetBenchmarking(true);
    FApp::SetFixedDeltaTime(Config::get<double>("SIMULATION_CONTROLLER.PHYSICS.SIMULATION_STEP_TIME_SECONDS"));

    // pause the game
    UGameplayStatics::SetGamePaused(world_, true);

    // create Agent
    if (Config::get<std::string>("SIMULATION_CONTROLLER.AGENT") == "CameraAgent") {
        agent_ = std::make_unique<CameraAgent>(world_);
    } else if (Config::get<std::string>("SIMULATION_CONTROLLER.AGENT") == "SphereAgent") {
        agent_ = std::make_unique<SphereAgent>(world_);
    } else {
        ASSERT(false);
    }
    ASSERT(agent_);

    // create Task
    if (Config::get<std::string>("SIMULATION_CONTROLLER.TASK") == "ImitationLearningTask") {
        task_ = std::make_unique<ImitationLearningTask>(world_);
    } else if (Config::get<std::string>("SIMULATION_CONTROLLER.TASK") == "NullTask") {
        task_ = std::make_unique<NullTask>();
    } else if (Config::get<std::string>("SIMULATION_CONTROLLER.TASK") == "PointGoalNavTask") {
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

    // initialize RPC server
    rpc_server_ = std::make_unique<RpcServer>(
        Config::get<std::string>("SIMULATION_CONTROLLER.IP"),
        Config::get<int>("SIMULATION_CONTROLLER.PORT"));
    ASSERT(rpc_server_);

    bindFunctionsToRpcServer();

    int num_worker_threads = 1;
    rpc_server_->runAsync(num_worker_threads);

    has_world_begin_play_executed_ = true;
}

void SimulationController::worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    std::cout << "[SPEAR | SimulationController.cpp] SimulationController::worldCleanupEventHandler" << std::endl;

    ASSERT(world);

    // We only need to perform any additional steps if the world being cleaned up is the world we cached in our world_ member variable.
    if (world == world_) {

        // The worldCleanupEventHandler(...) function is called for all worlds, but some local state (such as rpc_server_ and agent_)
        // is initialized only when worldBeginPlayEventHandler(...) is called for a particular world. So we check if worldBeginPlayEventHandler(...)
        // has been executed.
        if (has_world_begin_play_executed_) {
            has_world_begin_play_executed_ = false;

            ASSERT(rpc_server_);
            rpc_server_->stopRunAsync();
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
        if (Config::get<std::string>("SIMULATION_CONTROLLER.INTERACTION_MODE") == "programmatic") {
            world_->OnWorldBeginPlay.Remove(world_begin_play_delegate_handle_);
            world_begin_play_delegate_handle_.Reset();
        }

        // clear cached world_ pointer
        world_ = nullptr;
    }
}

void SimulationController::beginFrameEventHandler()
{
    // if beginTick() has indicated that we should advance the simulation
    if (frame_state_ == FrameState::RequestPreTick) {

        // update frame state, allow begin_tick() to finish executing
        frame_state_ = FrameState::ExecutingPreTick;
        frame_state_executing_pre_tick_promise_.set_value();

        // unpause the game
        UGameplayStatics::SetGamePaused(world_, false);

        // execute all pre-tick synchronous work, wait here for tick() to unblock
        rpc_server_->runSync();

        // execute task-specific beginFrame() work
        task_->beginFrame();

        // update local state
        frame_state_ = FrameState::ExecutingTick;
    }
}

void SimulationController::endFrameEventHandler()
{
    // if we are currently advancing the simulation
    if (frame_state_ == FrameState::ExecutingTick) {

        // update frame state, allow tick() to finish executing
        frame_state_ = FrameState::ExecutingPostTick;
        frame_state_executing_post_tick_promise_.set_value();

        // execute task-specific endFrame() work
        task_->endFrame();

        // execute all post-tick synchronous work, wait here for endTick() to unblock
        rpc_server_->runSync();

        // pause the game
        UGameplayStatics::SetGamePaused(world_, true);

        // update frame state, allow endTick() to finish executing
        frame_state_ = FrameState::Idle;
        frame_state_idle_promise_.set_value();
    }
}

void SimulationController::bindFunctionsToRpcServer()
{
    rpc_server_->bindAsync("ping", []() -> std::string {
        return "SimulationController received a call to ping()...";
    });

    rpc_server_->bindAsync("getByteOrder", []() -> std::string {
        uint32_t dummy = 0x01020304;
        return (reinterpret_cast<char*>(&dummy)[3] == 1) ? "little" : "big";
    });

    rpc_server_->bindAsync("requestClose", []() -> void {
        bool immediate_shutdown = false;
        FGenericPlatformMisc::RequestExit(immediate_shutdown);
    });

    rpc_server_->bindAsync("beginTick", [this]() -> void {
        ASSERT(frame_state_ == FrameState::Idle);

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

        ASSERT(frame_state_ == FrameState::ExecutingPreTick);
    });

    rpc_server_->bindAsync("tick", [this]() -> void {
        ASSERT(frame_state_ == FrameState::ExecutingPreTick);

        // allow beginFrameEventHandler() to finish executing, wait here until frame_state == FrameState::ExecutingPostTick
        rpc_server_->requestStopRunSync();
        frame_state_executing_post_tick_future_.wait();

        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
    });

    rpc_server_->bindAsync("endTick", [this]() -> void {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);

        // allow endFrameEventHandler() to finish executing, wait here until frame_state == FrameState::Idle
        rpc_server_->requestStopRunSync();
        frame_state_idle_future_.wait();

        ASSERT(frame_state_ == FrameState::Idle);
    });

    rpc_server_->bindAsync("getActionSpace", [this]() -> std::map<std::string, ArrayDesc> {
        ASSERT(agent_);
        return agent_->getActionSpace();
    });

    rpc_server_->bindAsync("getObservationSpace", [this]() -> std::map<std::string, ArrayDesc> {
        ASSERT(agent_);
        return agent_->getObservationSpace();
    });

    rpc_server_->bindAsync("getAgentStepInfoSpace", [this]() -> std::map<std::string, ArrayDesc> {
        ASSERT(agent_);
        return agent_->getStepInfoSpace();
    });

    rpc_server_->bindAsync("getTaskStepInfoSpace", [this]() -> std::map<std::string, ArrayDesc> {
        ASSERT(task_);
        return task_->getStepInfoSpace();
    });

    rpc_server_->bindSync("applyAction", [this](std::map<std::string, std::vector<uint8_t>> action) -> void {
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

    rpc_server_->bindSync("isAgentReady", [this]() -> bool {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(agent_);
        return agent_->isReady();
    });

    rpc_server_->bindSync("isTaskReady", [this]() -> bool {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(task_);
        return task_->isReady();
    });
}

IMPLEMENT_MODULE(SimulationController, SimulationController)
