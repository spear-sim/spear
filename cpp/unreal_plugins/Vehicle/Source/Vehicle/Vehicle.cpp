//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "Vehicle/Vehicle.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistrar.h"

void Vehicle::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();
}

void Vehicle::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();
}

// use if module does not implement any Unreal classes
// IMPLEMENT_MODULE(Vehicle, Vehicle);

// use if module implements any Unreal classes
IMPLEMENT_GAME_MODULE(Vehicle, Vehicle);
