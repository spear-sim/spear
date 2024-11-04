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
}

ASpInitializeWorldManager::~ASpInitializeWorldManager()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpInitializeWorldManager::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::BeginPlay();

    // Set physics settings
    if (bSetPhysicsSettings) {
        SP_LOG("Setting physics settings...");

        SP_LOG("Old physics settings: ");
        SP_LOG_NO_PREFIX(Unreal::getObjectPropertiesAsString(UPhysicsSettings::Get()));

        SP_LOG("New physics settings: ");
        std::string sp_physics_settings_str = Unreal::getObjectPropertiesAsString(&SpPhysicsSettings, FSpPhysicsSettings::StaticStruct());
        SP_LOG_NO_PREFIX(sp_physics_settings_str);

        Unreal::setObjectPropertiesFromString(UPhysicsSettings::Get(), sp_physics_settings_str);
    }

    // Set fixed delta time
    if (bSetFixedDeltaTime) {    
        SP_LOG("Setting fixed delta time...");

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

    // Set game paused
    if (bSetGamePaused) {
        SP_LOG("Setting game paused...");
        SP_LOG("Old game paused: ", UGameplayStatics::IsGamePaused(GetWorld()));
        SP_LOG("New game paused: ", GamePaused);
        UGameplayStatics::SetGamePaused(GetWorld(), GamePaused);
    }

    // Execute console commands
    if (bExecuteConsoleCommands) {
        SP_LOG("Executing console commands...");
        SP_ASSERT(GEngine);
        for (auto& command : ConsoleCommands) {
            GEngine->Exec(GetWorld(), *command);
        }
    }
}
