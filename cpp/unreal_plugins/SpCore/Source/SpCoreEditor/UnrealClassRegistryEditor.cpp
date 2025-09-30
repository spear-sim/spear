//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCoreEditor/UnrealClassRegistryEditor.h"

#include <string>

#include "SpCore/FuncRegistry.h"

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
// Registrys for getting subsystems using a class name instead of template parameters
//

FuncRegistry<UEditorSubsystem*> g_get_editor_subsystem_by_type_func_registrar;

//
// Get editor subsystem using a class name instead of template parameters
//

UEditorSubsystem* UnrealClassRegistryEditor::getEditorSubsystemByType(const std::string& class_name) {
    return g_get_editor_subsystem_by_type_func_registrar.call(class_name);
}
