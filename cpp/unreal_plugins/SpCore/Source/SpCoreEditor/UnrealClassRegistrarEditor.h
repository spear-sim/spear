//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>

#include <HAL/Platform.h> // SPCOREEDITOR_API

#include "SpCore/FuncRegistrar.h"
#include "SpCore/UnrealClassRegistrar.h"

#include "SpCoreEditor/UnrealEditor.h"

class UEditorSubsystem;

//
// We need the variables below to be globals because they are referenced in templated code. This requirement
// arises because templated code gets compiled at each call site. If a call site is in a different module
// (i.e., outside of SpCore), and it references a static variable, then the module will get its own local
// copy of the static variable. This behavior only happens on Clang, because MSVC has a different methodology
// for handling static variables in shared libraries. See the link below for details:
//     https://stackoverflow.com/questions/31495877/i-receive-different-results-on-unix-and-win-when-use-static-members-with-static
//

//
// Registrars for getting subsystems using a class name instead of template parameters
//

extern SPCOREEDITOR_API FuncRegistrar<UEditorSubsystem*> g_get_editor_subsystem_by_type_func_registrar;

class SPCOREEDITOR_API UnrealClassRegistrarEditor
{
public:
    UnrealClassRegistrarEditor() = delete;
    ~UnrealClassRegistrarEditor() = delete;

    //
    // Get editor subsystem using a class name instead of template parameters
    //

	static UEditorSubsystem* getEditorSubsystemByType(const std::string& class_name);

    //
    // Register editor subsystem class
    //

    template <CEditorSubsystem TEditorSubsystem>
    static void registerEditorSubsystemClass(const std::string& class_name)
    {
        UnrealClassRegistrar::registerClassCommon<TEditorSubsystem>(class_name);

        g_get_editor_subsystem_by_type_func_registrar.registerFunc(
            class_name, []() -> UEditorSubsystem* {
                return UnrealEditor::getEditorSubsystemByType<TEditorSubsystem>();
            });
    }

    //
    // Unregister classes
    //

    template <CEditorSubsystem TEditorSubsystem>
    static void unregisterEditorSubsystemClass(const std::string& class_name)
    {
        UnrealClassRegistrar::unregisterClassCommon<TEditorSubsystem>(class_name);

        g_get_editor_subsystem_by_type_func_registrar.unregisterFunc(class_name);
    }
};
