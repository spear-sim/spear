//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <mutex> // std::lock_guard
#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/World.h>                // UWorld::InitializationValues
#include <HAL/IConsoleManager.h>         // IConsoleVariable
#include <Kismet/GameplayStatics.h>
#include <Misc/App.h>                    // FApp::GetDeltaTime

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/SpStableNameManager.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"
#include "SpServices/SpTypes.h"

class WorldRegistryService : public Service
{
public:
    WorldRegistryService() = delete;
    WorldRegistryService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &WorldRegistryService::postWorldInitializationHandler);
        world_initialized_actors_handle_ = FWorldDelegates::OnWorldInitializedActors.AddRaw(this, &WorldRegistryService::worldInitializedActorsHandler);
        world_begin_tear_down_handle_ = FWorldDelegates::OnWorldBeginTearDown.AddRaw(this, &WorldRegistryService::worldBeginTearDownHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &WorldRegistryService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("world_registry_service", "get_world_descs", [this]() -> std::map<std::string, SpWorldDesc> {
            std::lock_guard<std::mutex> lock(mutex_);
            return world_descs_;
        });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("world_registry_service", "remove_world", [this](uint64_t& world) -> void {
            SP_ASSERT(world);
            std::lock_guard<std::mutex> lock(mutex_);

            UWorld* world_ptr = toPtr<UWorld>(world);
            std::string name = Unreal::toStdString(world_ptr->GetPathName());
            SP_ASSERT(Std::containsKey(world_descs_, name));
            SpWorldDesc& desc = world_descs_.at(name);
            world_ptr->OnWorldBeginPlay.Remove(desc.world_begin_play_handle_);
            desc.world_begin_play_handle_.Reset();

            Std::remove(world_descs_, name);
        });
    }

    ~WorldRegistryService() override
    {
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);
        FWorldDelegates::OnWorldInitializedActors.Remove(world_initialized_actors_handle_);
        FWorldDelegates::OnWorldBeginTearDown.Remove(world_begin_tear_down_handle_);
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);

        post_world_initialization_handle_.Reset();
        world_initialized_actors_handle_.Reset();
        world_begin_tear_down_handle_.Reset();
        world_cleanup_handle_.Reset();
    }

private:
    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
    {
        SP_ASSERT(world);
        std::lock_guard<std::mutex> lock(mutex_);

        // if not a preview world (e.g., the StaticMesh viewer in the editor) and if not a short-lived temporary world (e.g., during application startup)
        if (!world->IsPreviewWorld() && GEngine->GetWorldContextFromWorld(world)) {
            SP_ASSERT(world->IsEditorWorld() || world->IsGameWorld());

            std::string name = Unreal::toStdString(world->GetPathName());
            SP_ASSERT(!Std::containsKey(world_descs_, name));

            SpWorldDesc desc;
            desc.world_ = world;
            desc.world_id_ = current_world_id_++;
            desc.is_editor_world_ = world->IsEditorWorld();
            desc.is_game_world_ = world->IsGameWorld();
            desc.is_playing_ = false;
            desc.world_begin_play_handle_ = world->OnWorldBeginPlay.AddLambda([this, world]() {
                worldBeginPlayHandler(world);
            });

            Std::insert(world_descs_, name, desc);
        }
    }

    void worldInitializedActorsHandler(const FActorsInitializedParams& params)
    {
        UWorld* world = params.World;
        SP_ASSERT(world);

        bool spawn_stable_name_manager = true;
        if (Config::isInitialized()) {
            spawn_stable_name_manager = Config::get<bool>("SP_SERVICES.WORLD_REGISTRY_SERVICE.SPAWN_SP_STABLE_NAME_MANAGER");
        }

        // We spawn a ASpStableNameManager if:
        //   - the config system is uninitialized or it is initialized and the flag tells us we can spawn; AND
        //   - we're not in a preview world and not in a short-lived temporary world (e.g., during application startup); AND
        //   - we're in a PIE game world or a standalone game world; AND
        //   - there isn't already a ASpStableNameManager in the world.

        if (spawn_stable_name_manager) {
            if (!world->IsPreviewWorld() && GEngine->GetWorldContextFromWorld(world)) {
                SP_ASSERT(world->IsEditorWorld() || world->IsGameWorld());
                if (world->IsGameWorld()) {
                    std::vector<ASpStableNameManager*> sp_stable_name_managers = UnrealUtils::findActorsByType<ASpStableNameManager>(world);
                    if (sp_stable_name_managers.empty()) {
                        AActor* sp_stable_name_manager = world->SpawnActor<ASpStableNameManager>();
                        UnrealUtils::setStableName(sp_stable_name_manager, "__SP_STABLE_NAME_MANAGER__");
                    }
                }
            }
        }
    }

    void worldBeginTearDownHandler(UWorld* world)
    {
        SP_ASSERT(world);
    }

    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
    {
        SP_ASSERT(world);
        std::lock_guard<std::mutex> lock(mutex_);

        std::string name = Unreal::toStdString(world->GetPathName());

        if (Std::containsKey(world_descs_, name)) {
            SpWorldDesc& desc = world_descs_.at(name);
            world->OnWorldBeginPlay.Remove(desc.world_begin_play_handle_);
            desc.world_begin_play_handle_.Reset();
            Std::remove(world_descs_, name);
        }

        //
        // If we're the last world to get cleaned up, then restore the skylight CVar.
        //

        if (Std::containsKey(skylight_state_, name)) {
            Std::remove(skylight_state_, name);
            if (skylight_state_.empty()) {
                restoreSkylightCVar();
            }
        }
    }

    void worldBeginPlayHandler(UWorld* world)
    {
        SP_ASSERT(world);
        std::lock_guard<std::mutex> lock(mutex_);

        std::string name = Unreal::toStdString(world->GetPathName());
        SP_ASSERT(Std::containsKey(world_descs_, name));
        SpWorldDesc& desc = world_descs_.at(name);
        desc.is_playing_ = true;

        //
        // Override game paused
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.WORLD_REGISTRY_SERVICE.OVERRIDE_GAME_PAUSED")) {
            bool game_paused = Config::get<bool>("SP_SERVICES.WORLD_REGISTRY_SERVICE.GAME_PAUSED");

            SP_LOG("    Overriding game paused for world: ", name);
            SP_LOG("    Old game paused: ", UGameplayStatics::IsGamePaused(desc.world_));
            SP_LOG("    New game paused: ", game_paused);

            UGameplayStatics::SetGamePaused(desc.world_, game_paused);
        }

        //
        // Force skylight update for game worlds. This is necessary to work around an intermittent bug where
        // some scenes sometimes appear too dark in standalone macOS builds.
        //

        if (desc.is_game_world_) {
            bool force_skylight_update = true;
            float max_duration_seconds = 1.0f;

            if (Config::isInitialized()) {
                force_skylight_update = Config::get<bool>("SP_SERVICES.WORLD_REGISTRY_SERVICE.FORCE_SKYLIGHT_UPDATE");
                max_duration_seconds = Config::get<float>("SP_SERVICES.WORLD_REGISTRY_SERVICE.FORCE_SKYLIGHT_UPDATE_MAX_DURATION_SECONDS");
            }

            if (force_skylight_update) {
                if (skylight_state_.empty()) {
                    SP_LOG("    Forcing skylight updates every frame for ", max_duration_seconds, " seconds...");
                    setSkylightCVar();
                }
                Std::insert(skylight_state_, name, max_duration_seconds);
            }
        }
    }

    void beginFrame() override
    {
        Service::beginFrame();

        //
        // Track how long we've been updating the skylight.
        //

        if (!skylight_state_.empty()) {
            float delta_seconds = FApp::GetDeltaTime();

            std::vector<std::string> completed;
            for (auto& [name, remaining] : skylight_state_) {
                remaining -= delta_seconds;
                if (remaining <= 0.0f) {
                    completed.push_back(name);
                }
            }

            for (const auto& name : completed) {
                Std::remove(skylight_state_, name);
            }

            if (skylight_state_.empty()) {
                restoreSkylightCVar();
            }
        }
    }

    void setSkylightCVar()
    {
        IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr("r.SkylightUpdateEveryFrame"));
        SP_ASSERT(cvar);
        skylight_previous_cvar_value_ = cvar->GetInt();
        cvar->Set(1);
        SP_LOG("    Old value of r.SkylightUpdateEveryFrame: ", skylight_previous_cvar_value_);
        SP_LOG("    New value of r.SkylightUpdateEveryFrame: 1");
    }

    void restoreSkylightCVar()
    {
        IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr("r.SkylightUpdateEveryFrame"));
        SP_ASSERT(cvar);
        cvar->Set(skylight_previous_cvar_value_);
        SP_LOG("    Setting r.SkylightUpdateEveryFrame: ", skylight_previous_cvar_value_);
    }

    std::mutex mutex_;
    int64_t current_world_id_ = 1;
    std::map<std::string, SpWorldDesc> world_descs_;

    // per-world remaining seconds for skylight update; cvar restored when map goes empty
    int skylight_previous_cvar_value_ = -1;
    std::map<std::string, float> skylight_state_;

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_initialized_actors_handle_;
    FDelegateHandle world_begin_tear_down_handle_;
    FDelegateHandle world_cleanup_handle_;
};
