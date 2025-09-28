//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts> // std::derived_from

#include <Editor.h>        // GEditor
#include <EditorSubsystem.h>
#include <HAL/Platform.h>  // SPCOREEDITOR_API
#include <UObject/Class.h> // UClass

#include "SpCore/Assert.h"
#include "SpCore/Unreal.h"

//
// Concepts for Unreal objects
//

template <typename TEditorSubsystem>
concept CEditorSubsystem =
    CSubsystem<TEditorSubsystem> &&
    std::derived_from<TEditorSubsystem, UEditorSubsystem>;

//
// General-purpose functions for working with Unreal objects.
//

class SPCOREEDITOR_API UnrealEditor
{
public:
    UnrealEditor() = delete;
    ~UnrealEditor() = delete;

    //
    // Get editor subsystem, uclass can't be const because we need to pass it to GetEngineSubsystemBase(...)
    //

    template <CEditorSubsystem TEditorSubsystem>
    static TEditorSubsystem* getEditorSubsystemByType()
    {
        SP_ASSERT(GEditor);
        return GEditor->GetEditorSubsystem<TEditorSubsystem>();
    }

    static UEditorSubsystem* getEditorSubsystemByClass(UClass* uclass)
    {
        SP_ASSERT(GEditor);
        return GEditor->GetEditorSubsystemBase(uclass);
    }
};
