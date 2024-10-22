//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/LegacyService.h"

#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // UWorld

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpServices/EngineService.h"

#include "SpServices/Legacy/Agent.h"
#include "SpServices/Legacy/CameraAgent.h"
#include "SpServices/Legacy/NullAgent.h"
#include "SpServices/Legacy/SphereAgent.h"
#include "SpServices/Legacy/UrdfRobotAgent.h"
#include "SpServices/Legacy/VehicleAgent.h"

#include "SpServices/Legacy/Task.h"
#include "SpServices/Legacy/ImitationLearningTask.h"
#include "SpServices/Legacy/NullTask.h"

#include "SpServices/Legacy/NavMesh.h"

void LegacyService::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    SP_LOG("World name: ", Unreal::toStdString(world->GetName()));

    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
        SP_LOG("Caching world...");
        SP_ASSERT(!world_);
        world_ = world;
        world_begin_play_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &LegacyService::worldBeginPlayHandler);
    }
}

void LegacyService::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    SP_LOG("World name: ", Unreal::toStdString(world->GetName()));

    if (world == world_) {
        SP_LOG("Clearing cached world...");

        if (has_world_begin_play_executed_) {
            has_world_begin_play_executed_ = false;

            SP_ASSERT(nav_mesh_);
            SP_ASSERT(task_);
            SP_ASSERT(agent_);

            nav_mesh_->cleanUpObjectReferences();
            task_->cleanUpObjectReferences();
            agent_->cleanUpObjectReferences();

            nav_mesh_ = nullptr;
            task_ = nullptr;
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
        if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT") == "NullAgent") {
            agent_ = std::make_unique<NullAgent>();
        } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT") == "CameraAgent") {
            agent_ = std::make_unique<CameraAgent>(world_);
        } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT") == "SphereAgent") {
            agent_ = std::make_unique<SphereAgent>(world_);
        } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT") == "UrdfRobotAgent") {
            agent_ = std::make_unique<UrdfRobotAgent>(world_);
        } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.AGENT") == "VehicleAgent") {
            agent_ = std::make_unique<VehicleAgent>(world_);
        } else {
            SP_ASSERT(false);
        }

        if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.TASK") == "NullTask") {
            task_ = std::make_unique<NullTask>();
        } else if (Config::get<std::string>("SP_SERVICES.LEGACY_SERVICE.TASK") == "ImitationLearningTask") {
            task_ = std::make_unique<ImitationLearningTask>(world_);
        } else {
            SP_ASSERT(false);
        }

    } else {
        agent_ = std::make_unique<NullAgent>();
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
