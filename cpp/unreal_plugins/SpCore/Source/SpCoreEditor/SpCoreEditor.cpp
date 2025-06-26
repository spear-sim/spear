//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCoreEditor/SpCoreEditor.h"

#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

// Unreal classes
#include <LevelEditorSubsystem.h>

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"

#include "SpCoreEditor/UnrealClassRegistrarEditor.h"

void SpCoreEditor::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();

    registerClasses();
}

void SpCoreEditor::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    unregisterClasses();
}

// Normally we would do the operations in registerClasses() and unregisterClasses(...) in the opposite order.
// But we make an exception here (i.e., we do the operations in the same order) to make it easier and less
// error-prone to register classes.

void SpCoreEditor::registerClasses()
{
    // Unreal classes
    UnrealClassRegistrarEditor::registerEditorSubsystemClass<ULevelEditorSubsystem>("ULevelEditorSubsystem");
}

void SpCoreEditor::unregisterClasses()
{
    // Unreal classes
    UnrealClassRegistrarEditor::unregisterEditorSubsystemClass<ULevelEditorSubsystem>("ULevelEditorSubsystem");
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_MODULE(SpCoreEditor, SpCoreEditor);
