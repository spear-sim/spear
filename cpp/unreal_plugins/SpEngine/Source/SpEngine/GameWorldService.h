//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/World.h>                // UWorld
#include <Kismet/GameplayStatics.h>

#include "SpCore/Assert.h"
#include "SpCore/Unreal.h"
#include "SpEngine/EntryPointBinder.h"

class GameWorldService {
public:
    GameWorldService() = delete;
    GameWorldService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &GameWorldService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &GameWorldService::worldCleanupHandler);

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_game_paused", [this](const bool& paused) -> void {
            SP_ASSERT(world_);
            UGameplayStatics::SetGamePaused(world_, paused);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "open_level", [this](const std::string& level_name) -> void {
            SP_ASSERT(world_);
            SP_LOG("Opening level: ", level_name);
            UGameplayStatics::OpenLevel(world_, Unreal::toFName(level_name));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_world_name", [this]() -> std::string {
            SP_ASSERT(world_);
            return Unreal::toStdString(world_->GetName());
        });
    }

    ~GameWorldService()
    {
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);

private:
    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;

    UWorld* world_ = nullptr;
};
