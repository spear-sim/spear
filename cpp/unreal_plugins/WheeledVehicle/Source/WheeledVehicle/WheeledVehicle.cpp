//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "WheeledVehicle/WheeledVehicle.h"

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"

void WheeledVehicle::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();

    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));
}

void WheeledVehicle::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();
}

IMPLEMENT_MODULE(WheeledVehicle, WheeledVehicle)
