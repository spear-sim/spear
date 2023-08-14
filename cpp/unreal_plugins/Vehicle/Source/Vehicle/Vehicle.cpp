//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "Vehicle/Vehicle.h"

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"

void Vehicle::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void Vehicle::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();
}

IMPLEMENT_MODULE(Vehicle, Vehicle)
