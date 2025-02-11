//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpComponents.h"

#include <ChaosVehicleMovementComponent.h>
#include <Modules/ModuleManager.h> // IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE
#include <WheeledVehiclePawn.h>

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistrar.h"

#include "SpComponents/SpDebugManager.h"
#include "SpComponents/SpHitEventManager.h"
#include "SpComponents/SpInitializeWorldManager.h"
#include "SpComponents/SpSceneCaptureComponent2D.h"

void SpComponents::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();

    // ChaosVehicles types (can't register in SpCore because SpCore is loaded before ChaosVehicles, and
    // therefore SpCore cannot list ChaosVehicles as a dependency)
    UnrealClassRegistrar::registerActorClass<AWheeledVehiclePawn>("AWheeledVehiclePawn");
    UnrealClassRegistrar::registerComponentClass<UChaosVehicleMovementComponent>("UChaosVehicleMovementComponent");

    // SpComponents types
    UnrealClassRegistrar::registerActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistrar::registerActorClass<ASpInitializeWorldManager>("ASpInitializeWorldManager");
    UnrealClassRegistrar::registerActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistrar::registerComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
}

void SpComponents::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    // ChaosVehicles types (can't unregister in SpCore because SpCore is loaded before ChaosVehicles, and
    // therefore SpCore cannot list ChaosVehicles as a dependency)
    UnrealClassRegistrar::unregisterActorClass<AWheeledVehiclePawn>("AWheeledVehiclePawn");
    UnrealClassRegistrar::unregisterComponentClass<UChaosVehicleMovementComponent>("UChaosVehicleMovementComponent");

    // SpComponents types
    UnrealClassRegistrar::unregisterActorClass<ASpDebugManager>("ASpDebugManager");
    UnrealClassRegistrar::unregisterActorClass<ASpInitializeWorldManager>("ASpInitializeWorldManager");
    UnrealClassRegistrar::unregisterActorClass<ASpHitEventManager>("ASpHitEventManager");
    UnrealClassRegistrar::unregisterComponentClass<USpSceneCaptureComponent2D>("USpSceneCaptureComponent2D");
}

// use if module does not implement any Unreal classes
// IMPLEMENT_MODULE(SpComponents, SpComponents);

// use if module implements any Unreal classes
IMPLEMENT_GAME_MODULE(SpComponents, SpComponents);
