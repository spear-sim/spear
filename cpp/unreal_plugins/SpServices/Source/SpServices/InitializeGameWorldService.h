//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <atomic>
#include <string>
#include <vector>

#include <Engine/Engine.h>       // GEngine
#include <Engine/World.h>        // UWorld::InitializationValues
#include <HAL/IConsoleManager.h> // IConsoleVariable
#include <Kismet/GameplayStatics.h>

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

class InitializeGameWorldService : public Service
{
public:
    InitializeGameWorldService() = delete;
    InitializeGameWorldService(CUnrealEntryPointBinder auto* unreal_entry_point_binder, Service::WorldFilter* world_filter) : Service("InitializeGameWorldService", world_filter)
    {
        std::string service_name = getWorldTypeName() + ".initialize_game_world_service";

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread(service_name, "is_initialized", [this]() -> bool {
            return initialized_;
        });
    }

protected:
    void postWorldInitialization(UWorld* world, const UWorld::InitializationValues initialization_values) override
    {
        SP_LOG_CURRENT_FUNCTION();

        Service::postWorldInitialization(world, initialization_values);
    }

    void worldCleanup(UWorld* world, bool session_ended, bool cleanup_resources) override
    {
        SP_LOG_CURRENT_FUNCTION();

        initialized_ = false;

        if (force_skylight_update_) {
            SP_LOG("    Setting r.SkylightUpdateEveryFrame: ", force_skylight_update_previous_cvar_value_);

            IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr("r.SkylightUpdateEveryFrame"));
            SP_ASSERT(cvar);
            cvar->Set(force_skylight_update_previous_cvar_value_);
            force_skylight_update_ = false;
            force_skylight_update_max_duration_seconds_ = -1.0f;
            force_skylight_update_previous_cvar_value_ = -1;
            force_skylight_update_duration_seconds_ = 0.0f;
        }

        SP_LOG("    Finished cleaning up.");

        Service::worldCleanup(world, session_ended, cleanup_resources);
    }

    void worldBeginPlay() override
    {
        SP_LOG_CURRENT_FUNCTION();

        Service::worldBeginPlay();

        //
        // Override game paused
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_GAME_WORLD_SERVICE.OVERRIDE_GAME_PAUSED")) {
            bool game_paused = Config::get<bool>("SP_SERVICES.INITIALIZE_GAME_WORLD_SERVICE.GAME_PAUSED");

            SP_LOG("    Overriding game paused...");
            SP_LOG("    Old game paused: ", UGameplayStatics::IsGamePaused(getWorld()));
            SP_LOG("    New game paused: ", game_paused);

            UGameplayStatics::SetGamePaused(getWorld(), game_paused);
        }

        //
        // Execute console commands
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_GAME_WORLD_SERVICE.EXECUTE_CONSOLE_COMMANDS")) {
            std::vector<std::string> console_commands = Config::get<std::vector<std::string>>("SP_SERVICES.INITIALIZE_GAME_WORLD_SERVICE.CONSOLE_COMMANDS");

            SP_LOG("    Executing console commands...");
            SP_ASSERT(GEngine);
            for (auto& cmd : console_commands) {
                SP_LOG("    Executing console command: ", cmd);
                GEngine->Exec(getWorld(), Unreal::toTCharPtr(cmd));
            }
        }

        //
        // Force skylight update. This is necessary to work around an intermittent bug where the apartment scene
        // sometimes appears too dark in standalone macOS builds.
        //

        force_skylight_update_ = true;
        force_skylight_update_max_duration_seconds_ = 1.0f;
        force_skylight_update_previous_cvar_value_ = -1;
        force_skylight_update_duration_seconds_ = 0.0f;

        if (Config::isInitialized()) {
            force_skylight_update_ = Config::get<bool>("SP_SERVICES.INITIALIZE_GAME_WORLD_SERVICE.FORCE_SKYLIGHT_UPDATE");
            force_skylight_update_max_duration_seconds_ = Config::get<float>("SP_SERVICES.INITIALIZE_GAME_WORLD_SERVICE.FORCE_SKYLIGHT_UPDATE_MAX_DURATION_SECONDS");
        }

        if (force_skylight_update_) {
            SP_LOG("    Forcing skylight updates every frame for ", force_skylight_update_max_duration_seconds_, " seconds...");

            IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr("r.SkylightUpdateEveryFrame"));
            SP_ASSERT(cvar);
            force_skylight_update_previous_cvar_value_ = cvar->GetInt();
            cvar->Set(1);

            SP_LOG("    Old value of r.SkylightUpdateEveryFrame: ", force_skylight_update_previous_cvar_value_);
            SP_LOG("    New value of r.SkylightUpdateEveryFrame: 1");
        } else {
            initialized_ = true;
            SP_LOG("    Finished initializing.");
        }
    }

    void beginFrame() override
    {
        Service::beginFrame();

        // Force skylight update. We need to keep the r.SkylightUpdateEveryFrame console variable switched on for
        // several frames to work around an intermittent bug where the apartment scene sometimes appears too dark
        // in standalone macOS builds.

        if (force_skylight_update_) {
            force_skylight_update_duration_seconds_ += getWorld()->GetDeltaSeconds();

            if (force_skylight_update_duration_seconds_ >= force_skylight_update_max_duration_seconds_) {
                SP_LOG_CURRENT_FUNCTION();
                SP_LOG("    Setting r.SkylightUpdateEveryFrame: ", force_skylight_update_previous_cvar_value_);

                IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr("r.SkylightUpdateEveryFrame"));
                SP_ASSERT(cvar);
                cvar->Set(force_skylight_update_previous_cvar_value_);
                force_skylight_update_ = false;
                force_skylight_update_max_duration_seconds_ = -1.0f;
                force_skylight_update_previous_cvar_value_ = -1;
                force_skylight_update_duration_seconds_ = 0.0f;

                initialized_ = true;
                SP_LOG("    Finished initializing.");
            }
        }
    }

private:
    // Initialization state
    std::atomic<bool> initialized_ = false;

    // State required to force skylight update
    bool force_skylight_update_ = false;
    float force_skylight_update_max_duration_seconds_ = -1.0f;
    int force_skylight_update_previous_cvar_value_ = -1;
    float force_skylight_update_duration_seconds_ = 0.0f;
};
