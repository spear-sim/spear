//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponentsEditor/SpComponentsEditor.h"

#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"

void SpComponentsEditor::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();
}

void SpComponentsEditor::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();
}

IMPLEMENT_MODULE(SpComponentsEditor, SpComponentsEditor)
