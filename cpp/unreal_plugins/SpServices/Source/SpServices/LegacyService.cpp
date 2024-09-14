//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/LegacyService.h"

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

#include "SpServices/EngineService.h"

#include "SpServices/Legacy/Agent.h"
#include "SpServices/Legacy/CameraAgent.h"
#include "SpServices/Legacy/ClassRegistrationUtils.h"
#include "SpServices/Legacy/ImitationLearningTask.h"
#include "SpServices/Legacy/NavMesh.h"
#include "SpServices/Legacy/NullAgent.h"
#include "SpServices/Legacy/NullTask.h"
#include "SpServices/Legacy/SphereAgent.h"
#include "SpServices/Legacy/Task.h"
#include "SpServices/Legacy/UrdfRobotAgent.h"
#include "SpServices/Legacy/VehicleAgent.h"


void LegacyService::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
        std::string scene_id = "";
        std::string map_id = "";

        if (Config::isInitialized()) {
            scene_id = Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.SCENE_ID");
            map_id = Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.MAP_ID");
        }

        std::string desired_world_path_name = "";
        std::string desired_level_name = "";
        if (scene_id != "") {
            if (map_id == "") {
                map_id = scene_id;
            }
            desired_level_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id;
        }

        bool open_level = scene_id != "" && scene_id != Unreal::toStdString(world->GetName());

        SP_LOG("scene_id:           ", scene_id);
        SP_LOG("map_id:             ", map_id);
        SP_LOG("desired_level_name: ", desired_level_name);
        SP_LOG("world->GetName():   ", Unreal::toStdString(world->GetName()));
        SP_LOG("open_level:         ", open_level);

        if (open_level) {
            SP_LOG("Opening level: ", desired_level_name);

            SP_ASSERT(!open_level_pending_);

            UGameplayStatics::OpenLevel(world, Unreal::toFName(desired_level_name));
            open_level_pending_ = true;

        } else {

            open_level_pending_ = false;

            SP_ASSERT(!world_);
            world_ = world;
            world_begin_play_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &LegacyService::worldBeginPlayHandler);
        }
    }
}

void LegacyService::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    if (world == world_) {

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

        world_->OnWorldBeginPlay.Remove(world_begin_play_handle_);
        world_begin_play_handle_.Reset();

        world_ = nullptr;
    }
}

void LegacyService::worldBeginPlayHandler()
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world_);

    if (Config::isInitialized()) {
        for (auto& command : Config::get<std::vector<std::string>>("SP_SERVICES.LEGACY_SERVICE.CUSTOM_UNREAL_CONSOLE_COMMANDS")) {
            GEngine->Exec(world_, *Unreal::toFString(command));
        }
    }

    UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
    if (Config::isInitialized()) {
        physics_settings->bEnableEnhancedDeterminism = Config::get<bool>("SP_SERVICES.LEGACY_SERVICE.PHYSICS.ENABLE_ENHANCED_DETERMINISM");
        physics_settings->bSubstepping = Config::get<bool>("SP_SERVICES.LEGACY_SERVICE.PHYSICS.ENABLE_SUBSTEPPING");
        physics_settings->MaxSubstepDeltaTime = Config::get<float>("SP_SERVICES.LEGACY_SERVICE.PHYSICS.MAX_SUBSTEP_DELTA_TIME");
        physics_settings->MaxSubsteps = Config::get<int32>("SP_SERVICES.LEGACY_SERVICE.PHYSICS.MAX_SUBSTEPS");
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
        step_time = Config::get<float>("SP_SERVICES.LEGACY_SERVICE.PHYSICS.SIMULATION_STEP_TIME");
    } else {
        step_time = 0.05;
    }

    if (physics_settings->bSubstepping) {
        float max_step_time_allowed_when_substepping = physics_settings->MaxSubstepDeltaTime * physics_settings->MaxSubsteps;
        SP_ASSERT(step_time <= max_step_time_allowed_when_substepping);
    }

    FApp::SetBenchmarking(true);
    FApp::SetFixedDeltaTime(step_time);

    UGameplayStatics::SetGamePaused(world_, true);

    if (Config::isInitialized()) {
        agent_ = std::unique_ptr<Agent>(ClassRegistrationUtils::create(Agent::s_class_registrar_, Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT"), world_));

        if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.TASK") == "NullTask") {
            task_ = std::make_unique<NullTask>();
        } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.TASK") == "ImitationLearningTask") {
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

    nav_mesh_ = std::make_unique<NavMesh>();
    SP_ASSERT(nav_mesh_);

    agent_->findObjectReferences(world_);
    task_->findObjectReferences(world_);
    nav_mesh_->findObjectReferences(world_);

    has_world_begin_play_executed_ = true;
}
