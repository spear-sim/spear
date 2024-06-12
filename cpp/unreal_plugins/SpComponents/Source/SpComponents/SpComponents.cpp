//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpComponents.h"

#include <ChaosWheeledVehicleMovementComponent.h>
#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistrar.h"

#include "SpComponents/CameraSensorComponent.h"
#include "SpComponents/SpHitEventActor.h"

void SpComponents::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();

    UnrealClassRegistrar::registerActorClass<ASpHitEventActor>("ASpHitEventActor");
    UnrealClassRegistrar::registerComponentClass<UCameraSensorComponent>("UCameraSensorComponent");
    UnrealClassRegistrar::registerComponentClass<UChaosWheeledVehicleMovementComponent>("UChaosWheeledVehicleMovementComponent");
}

void SpComponents::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    UnrealClassRegistrar::unregisterActorClass<ASpHitEventActor>("ASpHitEventActor");
    UnrealClassRegistrar::unregisterComponentClass<UCameraSensorComponent>("UCameraSensorComponent");
    UnrealClassRegistrar::unregisterComponentClass<UChaosWheeledVehicleMovementComponent>("UChaosWheeledVehicleMovementComponent");
}

IMPLEMENT_MODULE(SpComponents, SpComponents)
