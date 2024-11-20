//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpInitializeWorldManager.h"

#include <Engine/Engine.h> // GEngine
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <Misc/App.h>
#include <PhysicsEngine/PhysicsSettings.h>

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

    // Override physics settings.
    if (bOverridePhysicsSettings) {
        SP_LOG("Overriding physics settings...");

        SP_LOG("Old physics settings: ");
        SP_LOG_NO_PREFIX(Unreal::getObjectPropertiesAsString(UPhysicsSettings::Get()));

        SP_LOG("New physics settings: ");
        std::string sp_physics_settings_str = Unreal::getObjectPropertiesAsString(&SpPhysicsSettings, FSpPhysicsSettings::StaticStruct());
        SP_LOG_NO_PREFIX(sp_physics_settings_str);

        Unreal::setObjectPropertiesFromString(UPhysicsSettings::Get(), sp_physics_settings_str);
    }

    // Override fixed delta time.
    if (bOverrideFixedDeltaTime) {    
        SP_LOG("Overriding fixed delta time...");

        SP_LOG("Old benchmarking:     ", FApp::IsBenchmarking());
        SP_LOG("Old fixed delta time: ", FApp::GetFixedDeltaTime());

        UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
        if (physics_settings->bSubstepping) {
            double max_fixed_delta_time = physics_settings->MaxSubstepDeltaTime*physics_settings->MaxSubsteps;
            SP_LOG("Maximum allowed fixed delta time: ", max_fixed_delta_time);
            SP_ASSERT(FixedDeltaTime <= max_fixed_delta_time);
        }

        SP_LOG("New benchmarking:     ", true);
        SP_LOG("New fixed delta time: ", FixedDeltaTime);

        FApp::SetBenchmarking(true);
        FApp::SetFixedDeltaTime(FixedDeltaTime);
    }

    // Force update skylight. This is necessary to work around an intermittent bug where the apartment scene
    // sometimes appears too dark in standalone macOS builds.
    if (bForceUpdateSkylight) {
        SP_LOG("Forcing update of all skylights for ", ForceUpdateSkylightMaxDurationSeconds, " seconds...");

        IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
        force_update_skylight_previous_cvar_value_ = cvar->GetInt();
        cvar->Set(1);

        SP_LOG("Old value of r.SkylightUpdateEveryFrame: ", force_update_skylight_previous_cvar_value_);
        SP_LOG("New value of r.SkylightUpdateEveryFrame: 1");

        force_update_skylight_completed_ = false;
        force_update_skylight_duration_seconds_ = 0.0f;
    }

    // Override game paused.
    if (bOverrideGamePaused) {
        SP_LOG("Overriding game paused...");
        SP_LOG("Old game paused: ", UGameplayStatics::IsGamePaused(GetWorld()));
        SP_LOG("New game paused: ", GamePaused);
        UGameplayStatics::SetGamePaused(GetWorld(), GamePaused);
    }

    // Execute console commands.
    if (bExecuteConsoleCommands) {
        SP_LOG("Executing console commands...");
        SP_ASSERT(GEngine);
        for (auto& command : ConsoleCommands) {
            GEngine->Exec(GetWorld(), *command);
        }
    }
}

void ASpInitializeWorldManager::Tick(float delta_time)
{
    AActor::Tick(delta_time);

    // Force update skylight. We need to keep the r.SkylightUpdateEveryFrame switched on for several frames,
    // to work around an intermittent bug where the apartment scene sometimes appears too dark in standalone
    // macOS builds.
    if (bForceUpdateSkylight && !force_update_skylight_completed_) {
        force_update_skylight_duration_seconds_ += delta_time;

        if (force_update_skylight_duration_seconds_ >= ForceUpdateSkylightMaxDurationSeconds) {
            SP_LOG("Setting r.SkylightUpdateEveryFrame to ", force_update_skylight_previous_cvar_value_);

            IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.SkylightUpdateEveryFrame"));
            cvar->Set(force_update_skylight_previous_cvar_value_);

            force_update_skylight_previous_cvar_value_ = -1;
            force_update_skylight_duration_seconds_ = 0.0;
            force_update_skylight_completed_ = true;
        }
    }
}
