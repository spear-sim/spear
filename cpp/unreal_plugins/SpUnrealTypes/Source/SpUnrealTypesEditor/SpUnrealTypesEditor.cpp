//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypesEditor/SpUnrealTypesEditor.h"

#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"
#include "SpCore/UnrealClassRegistry.h"

// SpUnrealTypesEditor classes
#include "SpUnrealTypesEditor/SpBlueprintEditorUtils.h"
#include "SpUnrealTypesEditor/SpMovieSceneEventUtils.h"
#include "SpUnrealTypesEditor/SpMovieSceneSequenceEditor.h"
#include "SpUnrealTypesEditor/SpUnrealEdEngine.h"

void SpUnrealTypesEditor::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();

    registerClasses();
}

void SpUnrealTypesEditor::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    unregisterClasses();
}

// Normally we would do the operations in registerClasses() and unregisterClasses(...) in the opposite order.
// But we make an exception here (i.e., we do the operations in the same order) to make it easier and less
// error-prone to register classes.

void SpUnrealTypesEditor::registerClasses() const
{
    // SpUnrealTypes classes
    UnrealClassRegistry::registerClass<USpBlueprintEditorUtils>("USpBlueprintEditorUtils");
    UnrealClassRegistry::registerClass<USpMovieSceneEventUtils>("USpMovieSceneEventUtils");
    UnrealClassRegistry::registerClass<USpMovieSceneSequenceEditor>("USpMovieSceneSequenceEditor");
    UnrealClassRegistry::registerClass<USpUnrealEdEngine>("USpUnrealEdEngine");
}

void SpUnrealTypesEditor::unregisterClasses() const
{
    // SpUnrealTypes classes
    UnrealClassRegistry::unregisterClass<USpBlueprintEditorUtils>("USpBlueprintEditorUtils");
    UnrealClassRegistry::unregisterClass<USpMovieSceneEventUtils>("USpMovieSceneEventUtils");
    UnrealClassRegistry::unregisterClass<USpMovieSceneSequenceEditor>("USpMovieSceneSequenceEditor");
    UnrealClassRegistry::unregisterClass<USpUnrealEdEngine>("USpUnrealEdEngine");
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpUnrealTypesEditor, SpUnrealTypesEditor)
