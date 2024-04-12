//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/LegacyService.h"

#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // UWorld
#include <Kismet/GameplayStatics.h>
#include <Misc/App.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"
#include "SpEngine/EngineService.h"
#include "SpEngine/Legacy/Agent.h"
#include "SpEngine/Legacy/CameraAgent.h"
#include "SpEngine/Legacy/ClassRegistrationUtils.h"
#include "SpEngine/Legacy/ImitationLearningTask.h"
#include "SpEngine/Legacy/NavMesh.h"
#include "SpEngine/Legacy/NullAgent.h"
#include "SpEngine/Legacy/NullTask.h"
#include "SpEngine/Legacy/SphereAgent.h"
#include "SpEngine/Legacy/Task.h"
#include "SpEngine/Legacy/UrdfRobotAgent.h"
#include "SpEngine/Legacy/VehicleAgent.h"


void LegacyService::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

#if WITH_EDITOR // defined in an auto-generated header
    bool world_is_ready = world->IsGameWorld();
#else
    bool world_is_ready = GEngine->GetWorldContextFromWorld(world) != nullptr;
#endif

    if (world_is_ready) {

        std::string scene_id = "";
        std::string map_id = "";

        if (Config::isInitialized()) {
            scene_id = Config::get<std::string>("SP_ENGINE.LEGACY_SERVICE.SCENE_ID");
            map_id = Config::get<std::string>("SP_ENGINE.LEGACY_SERVICE.MAP_ID");
        }

        std::string desired_world_path_name = "";
        std::string desired_level_name = "";
        if (scene_id != "") {
            if (map_id == "") {
                map_id = scene_id;
            }
            desired_level_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id;
        }

        // if the current world is not the desired one, open the desired one
        bool open_level = scene_id != "" && scene_id != Unreal::toStdString(world->GetName());

        SP_LOG("scene_id:                ", scene_id);
        SP_LOG("map_id:                  ", map_id);
        SP_LOG("desired_level_name:      ", desired_level_name);
        SP_LOG("world->GetName():        ", Unreal::toStdString(world->GetName()));
        SP_LOG("open_level:              ", open_level);

        if (open_level) {
            SP_LOG("Opening level: ", desired_level_name);

            // if we're at this line of code and OpenLevel is already pending, it means we failed
            SP_ASSERT(!open_level_pending_);

            UGameplayStatics::OpenLevel(world, Unreal::toFName(desired_level_name));
            open_level_pending_ = true;

        } else {
            open_level_pending_ = false;

            // we expect worldCleanupHandler(...) to be called before a new world is created
            SP_ASSERT(!world_);

            // cache local reference to the UWorld
            world_ = world;

            // We need to defer initializing this handler until after we have a valid world_ pointer,
            // and we defer the rest of our initialization code until the OnWorldBeginPlay event.
            world_begin_play_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &LegacyService::worldBeginPlayHandler);
        }
    }
}

void LegacyService::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    // We only need to perform any additional steps if the world being cleaned up is the world we cached in our world_ member variable.
    if (world == world_) {

        // The worldCleanupHandler(...) function is called for all worlds, but some local state (such as rpc_server_ and agent_)
        // is initialized only when worldBeginPlayHandler(...) is called for a particular world. So we check if worldBeginPlayHandler(...)
        // has been executed.
        if (has_world_begin_play_executed_) {
            has_world_begin_play_executed_ = false;

            SP_ASSERT(nav_mesh_);
            nav_mesh_->cleanUpObjectReferences();
            nav_mesh_ = nullptr;

            SP_ASSERT(task_);
            task_->cleanUpObjectReferences();
            task_ = nullptr;

            SP_ASSERT(agent_);
            agent_->cleanUpObjectReferences();
            agent_ = nullptr;
        }

        // remove event handlers bound to this world before world gets cleaned up
        world_->OnWorldBeginPlay.Remove(world_begin_play_handle_);
        world_begin_play_handle_.Reset();

        // clear cached world_ pointer
        world_ = nullptr;
    }
}

void LegacyService::worldBeginPlayHandler()
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world_);

    // execute optional console commands from python client
    if (Config::isInitialized()) {
        for (auto& command : Config::get<std::vector<std::string>>("SP_ENGINE.LEGACY_SERVICE.CUSTOM_UNREAL_CONSOLE_COMMANDS")) {
            GEngine->Exec(world_, *Unreal::toFString(command));
        }
    }

    // set physics parameters
    UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
    if (Config::isInitialized()) {
        physics_settings->bEnableEnhancedDeterminism = Config::get<bool>("SP_ENGINE.LEGACY_SERVICE.PHYSICS.ENABLE_ENHANCED_DETERMINISM");
        physics_settings->bSubstepping = Config::get<bool>("SP_ENGINE.LEGACY_SERVICE.PHYSICS.ENABLE_SUBSTEPPING");
        physics_settings->MaxSubstepDeltaTime = Config::get<float>("SP_ENGINE.LEGACY_SERVICE.PHYSICS.MAX_SUBSTEP_DELTA_TIME");
        physics_settings->MaxSubsteps = Config::get<int32>("SP_ENGINE.LEGACY_SERVICE.PHYSICS.MAX_SUBSTEPS");
    } else {
        physics_settings->bEnableEnhancedDeterminism = true;
        physics_settings->bSubstepping = true;
        physics_settings->MaxSubstepDeltaTime = 0.01;
        physics_settings->MaxSubsteps = 100;
    }

    // Check that the physics substepping parameters match our desired simulation step time.
    // See https://carla.readthedocs.io/en/latest/adv_synchrony_timestep for more details.
    float step_time;
    if (Config::isInitialized()) {
        step_time = Config::get<float>("SP_ENGINE.LEGACY_SERVICE.PHYSICS.SIMULATION_STEP_TIME");
    } else {
        step_time = 0.05;
    }

    if (physics_settings->bSubstepping) {
        float max_step_time_allowed_when_substepping = physics_settings->MaxSubstepDeltaTime * physics_settings->MaxSubsteps;
        SP_ASSERT(step_time <= max_step_time_allowed_when_substepping);
    }

    // set fixed simulation step time in seconds, FApp::SetBenchmarking(true) is also needed to enable
    FApp::SetBenchmarking(true);
    FApp::SetFixedDeltaTime(step_time);

    // pause the game
    UGameplayStatics::SetGamePaused(world_, true);

    if (Config::isInitialized()) {
        // create Agent
        agent_ = std::unique_ptr<Agent>(ClassRegistrationUtils::create(Agent::s_class_registrar_, Config::get<std::string>("SP_ENGINE.LEGACY_SERVICE.AGENT"), world_));

        // create Task
        if (Config::get<std::string>("SP_ENGINE.LEGACY_SERVICE.TASK") == "NullTask") {
            task_ = std::make_unique<NullTask>();
        } else if (Config::get<std::string>("SP_ENGINE.LEGACY_SERVICE.TASK") == "ImitationLearningTask") {
            task_ = std::make_unique<ImitationLearningTask>(world_);
        } else {
            SP_ASSERT(false);
        }
    } else {
        agent_ = std::unique_ptr<Agent>(ClassRegistrationUtils::create(Agent::s_class_registrar_, "NullAgent", world_));
        task_ = std::make_unique<NullTask>();
    }
    SP_ASSERT(agent_);
    SP_ASSERT(task_);

    // create NavMesh
    nav_mesh_ = std::make_unique<NavMesh>();
    SP_ASSERT(nav_mesh_);

    // deferred initialization
    agent_->findObjectReferences(world_);
    task_->findObjectReferences(world_);
    nav_mesh_->findObjectReferences(world_);

    has_world_begin_play_executed_ = true;
}
