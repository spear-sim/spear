//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSim/SpearSim.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_PRIMARY_GAME_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"

void SpearSim::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();
}

void SpearSim::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();
}

// must be used at least once per game, even if the module doesn't implement any Unreal classes
IMPLEMENT_PRIMARY_GAME_MODULE(SpearSim, SpearSim, "SpearSim"); // ModuleImplClass, ModuleName, DEPRECATED_GameName
