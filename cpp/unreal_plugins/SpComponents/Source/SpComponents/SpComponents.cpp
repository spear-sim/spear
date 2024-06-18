//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpComponents.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"

void SpComponents::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();
}

void SpComponents::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();
}

IMPLEMENT_MODULE(SpComponents, SpComponents)
