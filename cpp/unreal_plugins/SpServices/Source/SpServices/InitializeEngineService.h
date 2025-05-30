//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include <GameMapsSettings.h>
#include <GenericPlatform/GenericPlatformFile.h>
#include <Kismet/GameplayStatics.h>
#include <Misc/App.h>
#include <Misc/CoreDelegates.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

class InitializeEngineService : public Service {
public:
    InitializeEngineService()
    {
        // Mount PAK files. PAK files in default locations are mounted before SpCore::StartupModule(), so
        // there is no danger of attempting to mount PAK files too early. See the function below for guidance
        // on how to set pak_order in Engine/Source/Runtime/PakFile/Private/IPlatformFilePak.cpp.
        //
        //     int32 FPakPlatformFile::GetPakOrderFromPakFilePath(const FString& PakFilePath)
        //     {
        //         if (PakFilePath.StartsWith(FString::Printf(TEXT("%sPaks/%s-"), *FPaths::ProjectContentDir(), FApp::GetProjectName())))
        //             return 4;
        //         else if (PakFilePath.StartsWith(FPaths::ProjectContentDir()))
        //             return 3;
        //         else if (PakFilePath.StartsWith(FPaths::EngineContentDir()))
        //             return 2;
        //         else if (PakFilePath.StartsWith(FPaths::ProjectSavedDir()))
        //             return 1;
        //         return 0;
        //     }

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.MOUNT_PAK_FILES") && FCoreDelegates::MountPak.IsBound()) {
            std::vector<std::string> pak_files = Config::get<std::vector<std::string>>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.PAK_FILES");
            int32 pak_order = 0;

            SP_LOG("Mounting PAK files...");
            for (auto& pak_file : pak_files) {
                SP_LOG("Mounting PAK file: ", pak_file);
                IPakFile* pak = FCoreDelegates::MountPak.Execute(Unreal::toFString(pak_file), pak_order);
                SP_ASSERT(pak);
            }
        }

        // Override default game map settings. We want to do this early enough to avoid loading the default map
        // specified in Unreal's config system, but late enough that that the UGameMapSettings default object
        // has already been initialized.

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_GAME_DEFAULT_MAP")) {
            std::string game_default_map = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.GAME_DEFAULT_MAP");

            SP_LOG("Overriding game default map...");
            SP_LOG("Old game default map: ", Unreal::toStdString(UGameMapsSettings::GetGameDefaultMap()));
            SP_LOG("New game default map: ", game_default_map);

            UGameMapsSettings::SetGameDefaultMap(Unreal::toFString(game_default_map));
        }

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_GLOBAL_DEFAULT_GAME_MODE")) {
            std::string global_default_game_mode = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.GLOBAL_DEFAULT_GAME_MODE");

            SP_LOG("Overriding global default game mode...");
            SP_LOG("Old global default game mode: ", Unreal::toStdString(UGameMapsSettings::GetGlobalDefaultGameMode()));
            SP_LOG("New global default game mode: ", global_default_game_mode);

            UGameMapsSettings::SetGlobalDefaultGameMode(Unreal::toFString(global_default_game_mode));
        }
    }

    ~InitializeEngineService() override = default;

protected:
    void worldBeginPlay() override
    {
        Service::worldBeginPlay();

        //
        // Override game paused
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_GAME_PAUSED")) {
            bool game_paused = Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.GAME_PAUSED");

            SP_LOG("Overriding game paused...");
            SP_LOG("Old game paused: ", UGameplayStatics::IsGamePaused(getWorld()));
            SP_LOG("New game paused: ", game_paused);

            UGameplayStatics::SetGamePaused(getWorld(), game_paused);
        }

        //
        // Override benchmarking
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_BENCHMARKING")) {
            bool benchmarking = Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.BENCHMARKING");

            SP_LOG("Overriding benchmarking...");
            SP_LOG("Old benchmarking: ", FApp::IsBenchmarking());
            SP_LOG("New benchmarking: ", benchmarking);

            FApp::SetBenchmarking(benchmarking);
        }

        //
        // Override fixed delta time
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_FIXED_DELTA_TIME")) {
            float fixed_delta_time = Config::get<float>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.FIXED_DELTA_TIME");

            SP_LOG("Overriding fixed delta time...");
            SP_LOG("Old fixed delta time: ", FApp::GetFixedDeltaTime());
            SP_LOG("New fixed delta time: ", fixed_delta_time);

            FApp::SetFixedDeltaTime(fixed_delta_time);
        }

        //
        // Override physics settings
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_PHYSICS_SETTINGS")) {
            std::string physics_settings_str = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.PHYSICS_SETTINGS_STRING");

            SP_LOG("Overriding physics settings...");
            SP_LOG("Old physics settings: ");
            SP_LOG_NO_PREFIX(Unreal::getObjectPropertiesAsString(UPhysicsSettings::Get()));
            SP_LOG("New physics settings: ");
            SP_LOG_NO_PREFIX(physics_settings_str);

            Unreal::setObjectPropertiesFromString(UPhysicsSettings::Get(), physics_settings_str);
        }

        //
        // Validate physics settings
        //

        UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
        if (FApp::IsBenchmarking() && physics_settings->bSubstepping) {
            double max_fixed_delta_time = physics_settings->MaxSubstepDeltaTime*physics_settings->MaxSubsteps;

            SP_LOG("Validating physics settings...");
            SP_LOG("Current fixed delta time:         ", FApp::GetFixedDeltaTime());
            SP_LOG("Maximum allowed fixed delta time: ", max_fixed_delta_time);

            SP_ASSERT(FApp::GetFixedDeltaTime() <= max_fixed_delta_time);
        }

        //
        // Execute console commands
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.EXECUTE_CONSOLE_COMMANDS")) {
            std::vector<std::string> console_commands = Config::get<std::vector<std::string>>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.CONSOLE_COMMANDS");

            SP_LOG("Executing console commands...");
            SP_ASSERT(GEngine);
            for (auto& cmd : console_commands) {
                SP_LOG("Executing console command: ", cmd);
                GEngine->Exec(getWorld(), *Unreal::toFString(cmd));
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
            force_skylight_update_ = Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.FORCE_SKYLIGHT_UPDATE");
            force_skylight_update_max_duration_seconds_ = Config::get<float>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.FORCE_SKYLIGHT_UPDATE_MAX_DURATION_SECONDS");
        }

        if (force_skylight_update_) {
            SP_LOG("Forcing skylight updates every frame for ", force_skylight_update_max_duration_seconds_, " seconds...");

            IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
            force_skylight_update_previous_cvar_value_ = cvar->GetInt();
            cvar->Set(1);

            SP_LOG("Old value of r.SkylightUpdateEveryFrame: ", force_skylight_update_previous_cvar_value_);
            SP_LOG("New value of r.SkylightUpdateEveryFrame: 1");
        }
    }

    void worldCleanup(UWorld* world, bool session_ended, bool cleanup_resources) override
    {
        if (force_skylight_update_) {
            SP_LOG("Setting r.SkylightUpdateEveryFrame to ", force_skylight_update_previous_cvar_value_);

            IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
            cvar->Set(force_skylight_update_previous_cvar_value_);

            force_skylight_update_ = false;
            force_skylight_update_max_duration_seconds_ = -1.0f;
            force_skylight_update_previous_cvar_value_ = -1;
            force_skylight_update_duration_seconds_ = 0.0f;
        }

        Service::worldCleanup(world, session_ended, cleanup_resources);
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
                SP_LOG("Setting r.SkylightUpdateEveryFrame to ", force_skylight_update_previous_cvar_value_);

                IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
                cvar->Set(force_skylight_update_previous_cvar_value_);

                force_skylight_update_ = false;
                force_skylight_update_max_duration_seconds_ = -1.0f;
                force_skylight_update_previous_cvar_value_ = -1;
                force_skylight_update_duration_seconds_ = 0.0f;
            }
        }
    }

private:

    //
    // State required to force skylight update.
    //

    bool force_skylight_update_ = false;
    float force_skylight_update_max_duration_seconds_ = -1.0f;
    int force_skylight_update_previous_cvar_value_ = -1;
    float force_skylight_update_duration_seconds_ = 0.0f;
};
