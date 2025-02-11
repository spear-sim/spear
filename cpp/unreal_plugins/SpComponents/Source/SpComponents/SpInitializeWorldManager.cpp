//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpInitializeWorldManager.h"

#include <filesystem>
#include <ranges> // std::views::transform

#include <Engine/Engine.h>      // GEngine
#include <Engine/EngineTypes.h> // EEndPlayReason
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <Misc/App.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

ASpInitializeWorldManager::ASpInitializeWorldManager()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = true;
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

ASpInitializeWorldManager::~ASpInitializeWorldManager()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpInitializeWorldManager::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::BeginPlay();

    Initialize();
}

void ASpInitializeWorldManager::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

    if (force_skylight_update_ && !force_skylight_update_completed_) {
        SP_LOG("Setting r.SkylightUpdateEveryFrame to ", force_skylight_update_previous_cvar_value_);

        IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
        cvar->Set(force_skylight_update_previous_cvar_value_);

        force_skylight_update_ = false;
        force_skylight_update_completed_ = false;
        force_skylight_update_previous_cvar_value_ = -1;
        force_skylight_update_max_duration_seconds_ = -1.0f;
        force_skylight_update_duration_seconds_ = 0.0f;
    }

    if (initialize_config_system_) {
        SP_LOG("Terminating config system...");
        initialize_config_system_ = false;
        Config::terminate();
    }

    AActor::EndPlay(end_play_reason);
}

void ASpInitializeWorldManager::Tick(float delta_time)
{
    AActor::Tick(delta_time);

    // Force skylight update. We need to keep the r.SkylightUpdateEveryFrame console variable switched on for
    // several frames to work around an intermittent bug where the apartment scene sometimes appears too dark
    // in standalone macOS builds.
    if (force_skylight_update_ && !force_skylight_update_completed_) {
        force_skylight_update_duration_seconds_ += delta_time;

        if (force_skylight_update_duration_seconds_ >= force_skylight_update_max_duration_seconds_) {
            SP_LOG("Setting r.SkylightUpdateEveryFrame to ", force_skylight_update_previous_cvar_value_);

            IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
            cvar->Set(force_skylight_update_previous_cvar_value_);

            force_skylight_update_ = false;
            force_skylight_update_completed_ = false;
            force_skylight_update_previous_cvar_value_ = -1;
            force_skylight_update_max_duration_seconds_ = -1.0f;
            force_skylight_update_duration_seconds_ = 0.0f;
        }
    }
}

void ASpInitializeWorldManager::Initialize()
{
    //
    // Initialize config system
    //

    std::string config_file;

    if (bInitializeConfigSystem) {
        config_file = Unreal::toStdString(ConfigFile);
        if (!Config::isInitialized() && std::filesystem::exists(config_file)) {
            initialize_config_system_ = true;
        }
    }

    if (initialize_config_system_) {
        SP_LOG("Initializing config system...");
        Config::initialize(config_file);
    }

    //
    // Override game paused
    //

    bool override_game_paused = false;
    bool game_paused = false;

    if (Config::isInitialized() && Config::get<bool>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.OVERRIDE_GAME_PAUSED")) {
        override_game_paused = true;
        game_paused = Config::get<bool>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.GAME_PAUSED");
    } else if (bOverrideGamePaused) {
        override_game_paused = true;
        game_paused = GamePaused;
    }

    if (override_game_paused) {
        SP_LOG("Overriding game paused...");
        SP_LOG("Old game paused: ", UGameplayStatics::IsGamePaused(GetWorld()));
        SP_LOG("New game paused: ", game_paused);
        UGameplayStatics::SetGamePaused(GetWorld(), game_paused);
    }

    //
    // Override benchmarking
    //

    bool override_benchmarking = false;
    bool benchmarking = false;

    if (Config::isInitialized() && Config::get<bool>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.OVERRIDE_BENCHMARKING")) {
        override_benchmarking = true;
        benchmarking = Config::get<bool>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.BENCHMARKING");
    } else if (bOverrideBenchmarking) {
        override_benchmarking = true;
        benchmarking = Benchmarking;
    }

    if (override_benchmarking) {
        SP_LOG("Overriding benchmarking...");
        SP_LOG("Old benchmarking: ", FApp::IsBenchmarking());
        SP_LOG("New benchmarking: ", benchmarking);
        FApp::SetBenchmarking(benchmarking);
    }

    //
    // Override fixed delta time
    //

    bool override_fixed_delta_time = false;
    float fixed_delta_time = -1.0f;

    if (Config::isInitialized() && Config::get<bool>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.OVERRIDE_FIXED_DELTA_TIME")) {
        override_fixed_delta_time = true;
        fixed_delta_time = Config::get<float>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.FIXED_DELTA_TIME");
    } else if (bOverrideFixedDeltaTime) {
        override_fixed_delta_time = true;
        fixed_delta_time = FixedDeltaTime;
    }

    if (override_fixed_delta_time) {
        SP_LOG("Overriding fixed delta time...");
        SP_LOG("Old fixed delta time: ", FApp::GetFixedDeltaTime());
        SP_LOG("New fixed delta time: ", fixed_delta_time);
        FApp::SetFixedDeltaTime(fixed_delta_time);
    }

    //
    // Override physics settings
    //

    bool override_physics_settings = false;
    std::string sp_physics_settings_str;

    if (Config::isInitialized() && Config::get<bool>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.OVERRIDE_PHYSICS_SETTINGS")) {
        override_physics_settings = true;
        sp_physics_settings_str = Config::get<std::string>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.SP_PHYSICS_SETTINGS_STRING");
    } else if (bOverridePhysicsSettings) {
        override_physics_settings = true;
        sp_physics_settings_str = Unreal::getObjectPropertiesAsString(&SpPhysicsSettings, FSpPhysicsSettings::StaticStruct());
    }

    if (override_physics_settings) {
        SP_LOG("Overriding physics settings...");
        SP_LOG("Old physics settings: ");
        SP_LOG_NO_PREFIX(Unreal::getObjectPropertiesAsString(UPhysicsSettings::Get()));
        SP_LOG("New physics settings: ");
        SP_LOG_NO_PREFIX(sp_physics_settings_str);

        Unreal::setObjectPropertiesFromString(UPhysicsSettings::Get(), sp_physics_settings_str);        
    }

    //
    // Force skylight update. This is necessary to work around an intermittent bug where the apartment scene
    // sometimes appears too dark in standalone macOS builds.
    //

    if (Config::isInitialized() && Config::get<bool>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.FORCE_SKYLIGHT_UPDATE")) {
        force_skylight_update_ = true;
        force_skylight_update_max_duration_seconds_ = Config::get<float>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.FORCE_SKYLIGHT_UPDATE_MAX_DURATION_SECONDS");
    } else if (bForceSkylightUpdate) {
        force_skylight_update_ = true;
        force_skylight_update_max_duration_seconds_ = ForceSkylightUpdateMaxDurationSeconds;
    }

    if (force_skylight_update_) {
        SP_LOG("Forcing skylight updates every frame for ", force_skylight_update_max_duration_seconds_, " seconds...");

        IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
        force_skylight_update_previous_cvar_value_ = cvar->GetInt();
        cvar->Set(1);

        SP_LOG("Old value of r.SkylightUpdateEveryFrame: ", force_skylight_update_previous_cvar_value_);
        SP_LOG("New value of r.SkylightUpdateEveryFrame: 1");

        force_skylight_update_completed_ = false;
        force_skylight_update_duration_seconds_ = 0.0f;
    }

    //
    // Execute console commands
    //

    bool execute_console_commands = false;
    std::vector<std::string> console_commands;

    if (Config::isInitialized() && Config::get<bool>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.EXECUTE_CONSOLE_COMMANDS")) {
        execute_console_commands = true;
        console_commands = Config::get<std::vector<std::string>>("SP_COMPONENTS.SP_INITIALIZE_WORLD_MANAGER.CONSOLE_COMMANDS");
    } else if (bExecuteConsoleCommands) {
        execute_console_commands = true;
        console_commands = Std::toVector<std::string>(Unreal::toStdVector(ConsoleCommands) | std::views::transform([](auto cmd) { return Unreal::toStdString(cmd); }));
    }

    if (execute_console_commands) {
        SP_LOG("Executing console commands...");
        SP_ASSERT(GEngine);
        for (auto& cmd : console_commands) {
            GEngine->Exec(GetWorld(), *Unreal::toFString(cmd));
        }
    }

    //
    // Validate physics settings
    //

    SP_LOG("Validating physics settings...");
    UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
    if (FApp::IsBenchmarking() && physics_settings->bSubstepping) {
        double max_fixed_delta_time = physics_settings->MaxSubstepDeltaTime*physics_settings->MaxSubsteps;
        SP_LOG("Current fixed delta time:         ", FApp::GetFixedDeltaTime());
        SP_LOG("Maximum allowed fixed delta time: ", max_fixed_delta_time);
        SP_ASSERT(FApp::GetFixedDeltaTime() <= max_fixed_delta_time);
    }
}
