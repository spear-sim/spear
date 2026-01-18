//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCoreEditor/UnrealEditor.h"

#include <Editor.h> // GEditor

#include "SpCore/Assert.h"

class UClass;
class UEditorSubsystem;

//
// Get editor subsystem, uclass can't be const because we need to pass it to GetEditorSubsystemBase(...)
//

UEditorSubsystem* UnrealEditor::getEditorSubsystemByClass(UClass* uclass)
{
    SP_ASSERT(GEditor);
    return GEditor->GetEditorSubsystemBase(uclass);
}
