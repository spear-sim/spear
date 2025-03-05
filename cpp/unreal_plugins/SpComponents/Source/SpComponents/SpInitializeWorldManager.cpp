//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpInitializeWorldManager.h"

#include <filesystem>
#include <ranges> // std::views::transform
#include <string>

#include <Engine/Engine.h>      // GEngine
#include <Engine/EngineTypes.h> // EEndPlayReason
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <Misc/App.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
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

    //
    // Initialize config system
    //

    if (bInitializeConfigSystem) {
        std::string config_file = Unreal::toStdString(ConfigFile);
        if (!Config::isInitialized() && std::filesystem::exists(config_file)) {
            SP_LOG("Initializing config system...");
            Config::initialize(config_file);
            initialize_config_system_ = true;
        }
    }

    //
    // Override game paused
    //

    if (bOverrideGamePaused) {
        SP_LOG("Overriding game paused...");
        SP_LOG("Old game paused: ", UGameplayStatics::IsGamePaused(GetWorld()));
        SP_LOG("New game paused: ", GamePaused);

        UGameplayStatics::SetGamePaused(GetWorld(), GamePaused);
    }

    //
    // Override benchmarking
    //

    if (bOverrideBenchmarking) {
        SP_LOG("Overriding benchmarking...");
        SP_LOG("Old benchmarking: ", FApp::IsBenchmarking());
        SP_LOG("New benchmarking: ", Benchmarking);
        FApp::SetBenchmarking(Benchmarking);
    }

    //
    // Override fixed delta time
    //

    if (bOverrideFixedDeltaTime) {
        SP_LOG("Overriding fixed delta time...");
        SP_LOG("Old fixed delta time: ", FApp::GetFixedDeltaTime());
        SP_LOG("New fixed delta time: ", FixedDeltaTime);
        FApp::SetFixedDeltaTime(FixedDeltaTime);
    }

    //
    // Override physics settings
    //

    if (bOverridePhysicsSettings) {
        std::string physics_settings_str = Unreal::getObjectPropertiesAsString(&PhysicsSettings, FSpPhysicsSettings::StaticStruct());

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

    if (bExecuteConsoleCommands) {
        std::vector<std::string> console_commands = Std::toVector<std::string>(Unreal::toStdVector(ConsoleCommands) | std::views::transform([](auto cmd) { return Unreal::toStdString(cmd); }));

        SP_LOG("Executing console commands...");
        SP_ASSERT(GEngine);
        for (auto& cmd : console_commands) {
            GEngine->Exec(GetWorld(), *Unreal::toFString(cmd));
        }
    }

    //
    // Force skylight update. This is necessary to work around an intermittent bug where the apartment scene
    // sometimes appears too dark in standalone macOS builds.
    //

    if (bForceSkylightUpdate) {

        force_skylight_update_ = true;
        force_skylight_update_previous_cvar_value_ = -1;
        force_skylight_update_duration_seconds_ = 0.0f;

        SP_LOG("Forcing skylight updates every frame for ", ForceSkylightUpdateMaxDurationSeconds, " seconds...");

        IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
        force_skylight_update_previous_cvar_value_ = cvar->GetInt();
        cvar->Set(1);

        SP_LOG("Old value of r.SkylightUpdateEveryFrame: ", force_skylight_update_previous_cvar_value_);
        SP_LOG("New value of r.SkylightUpdateEveryFrame: 1");
    }
}

void ASpInitializeWorldManager::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

    if (force_skylight_update_) {
        SP_LOG("Setting r.SkylightUpdateEveryFrame to ", force_skylight_update_previous_cvar_value_);

        IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
        cvar->Set(force_skylight_update_previous_cvar_value_);

        force_skylight_update_ = false;
        force_skylight_update_previous_cvar_value_ = -1;
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
    if (force_skylight_update_) {
        force_skylight_update_duration_seconds_ += delta_time;

        if (force_skylight_update_duration_seconds_ >= ForceSkylightUpdateMaxDurationSeconds) {
            SP_LOG("Setting r.SkylightUpdateEveryFrame to ", force_skylight_update_previous_cvar_value_);

            IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
            cvar->Set(force_skylight_update_previous_cvar_value_);

            force_skylight_update_ = false;
            force_skylight_update_previous_cvar_value_ = -1;
            force_skylight_update_duration_seconds_ = 0.0f;
        }
    }
}
