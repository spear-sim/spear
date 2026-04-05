//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <mutex> // std::lock_guard
#include <string>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/World.h>                // UWorld::InitializationValues
#include <Kismet/GameplayStatics.h>

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
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
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &WorldRegistryService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("world_registry_service", "get_world_descs", [this]() -> std::map<std::string, SpWorldDesc> {
            std::lock_guard<std::mutex> lock(mutex_);
            return world_descs_;
        });

        unreal_entry_point_binder->bindFuncToExecuteOnGameThread("world_registry_service", "remove_world", [this](uint64_t& world) -> void {
            SP_ASSERT(world);
            std::lock_guard<std::mutex> lock(mutex_);

            UWorld* world_ptr = toPtr<UWorld>(world);
            std::string name = Unreal::toStdString(world_ptr->GetName());
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
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);

        post_world_initialization_handle_.Reset();
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
    }

    std::mutex mutex_;
    int64_t current_world_id_ = 1;
    std::map<std::string, SpWorldDesc> world_descs_;

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;
};
