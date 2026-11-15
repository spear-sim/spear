//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpInitializeWorldManager.h"

#include <filesystem>
#include <string>

#include <Engine/EngineTypes.h> // EEndPlayReason
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <Misc/App.h>
#include <PhysicsEngine/PhysicsSettings.h>

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

ASpInitializeWorldManager::ASpInitializeWorldManager()
{
    SP_LOG_CURRENT_FUNCTION();
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
            SP_LOG("    Initializing config system...");
            Config::initialize(config_file);
            initialized_config_system_ = true;
        }
    }

    //
    // Override game paused
    //

    if (bOverrideGamePaused) {
        SP_LOG("    Overriding game paused...");
        SP_LOG("    Old game paused: ", UGameplayStatics::IsGamePaused(GetWorld()));
        SP_LOG("    New game paused: ", GamePaused);
        UGameplayStatics::SetGamePaused(GetWorld(), GamePaused);
    }

    //
    // Override benchmarking
    //

    if (bOverrideBenchmarking) {
        SP_LOG("    Overriding benchmarking...");
        SP_LOG("    Old benchmarking: ", FApp::IsBenchmarking());
        SP_LOG("    New benchmarking: ", Benchmarking);
        saved_benchmarking_ = FApp::IsBenchmarking();
        FApp::SetBenchmarking(Benchmarking);
    }

    //
    // Override fixed delta time
    //

    if (bOverrideFixedDeltaTime) {
        SP_LOG("    Overriding fixed delta time...");
        SP_LOG("    Old fixed delta time: ", FApp::GetFixedDeltaTime());
        SP_LOG("    New fixed delta time: ", FixedDeltaTime);
        saved_fixed_delta_time_ = FApp::GetFixedDeltaTime();
        FApp::SetFixedDeltaTime(FixedDeltaTime);
    }

    //
    // Override physics settings
    //

    if (bOverridePhysicsSettings) {
        std::string physics_settings_str = UnrealUtils::getObjectPropertiesAsString(&PhysicsSettings, FSpPhysicsSettings::StaticStruct());
        saved_physics_settings_str_ = UnrealUtils::getObjectPropertiesAsString(UPhysicsSettings::Get());

        SP_LOG("    Overriding physics settings...");
        SP_LOG("    Old physics settings: ");
        SP_LOG_NO_PREFIX(saved_physics_settings_str_);
        SP_LOG("    New physics settings: ");
        SP_LOG_NO_PREFIX(physics_settings_str);

        UnrealUtils::setObjectPropertiesFromString(UPhysicsSettings::Get(), physics_settings_str);
    }

    //
    // Validate physics settings
    //

    UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
    if (FApp::IsBenchmarking() && physics_settings->bSubstepping) {
        double max_fixed_delta_time = physics_settings->MaxSubstepDeltaTime*physics_settings->MaxSubsteps;

        SP_LOG("    Validating physics settings...");
        SP_LOG("    Current fixed delta time:         ", FApp::GetFixedDeltaTime());
        SP_LOG("    Maximum allowed fixed delta time: ", max_fixed_delta_time);

        SP_ASSERT(FApp::GetFixedDeltaTime() <= max_fixed_delta_time);
    }
}

void ASpInitializeWorldManager::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

    // FApp and UPhysicsSettings are global, so we restore them here to avoid leaking our overrides into the editor
    // after a play-in-editor session ends. We don't need to restore the game paused state because it is per-world.

    if (bOverridePhysicsSettings) {
        SP_LOG("    Restoring physics settings...");
        UnrealUtils::setObjectPropertiesFromString(UPhysicsSettings::Get(), saved_physics_settings_str_);
        saved_physics_settings_str_.clear();
    }

    if (bOverrideFixedDeltaTime) {
        SP_LOG("    Restoring fixed delta time: ", saved_fixed_delta_time_);
        FApp::SetFixedDeltaTime(saved_fixed_delta_time_);
        saved_fixed_delta_time_ = 0.0;
    }

    if (bOverrideBenchmarking) {
        SP_LOG("    Restoring benchmarking: ", saved_benchmarking_);
        FApp::SetBenchmarking(saved_benchmarking_);
        saved_benchmarking_ = false;
    }

    if (initialized_config_system_) {
        SP_LOG("    Terminating config system...");
        initialized_config_system_ = false;
        Config::terminate();
    }

    AActor::EndPlay(end_play_reason);
}
