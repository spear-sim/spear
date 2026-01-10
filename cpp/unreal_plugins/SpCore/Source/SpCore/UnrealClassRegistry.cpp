//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/UnrealClassRegistry.h"

#include <string>

#include "SpCore/FuncRegistry.h"

class UClass;
class USubsystem;
class UWorld;

//
// We need the variables below to be globals because they are referenced in templated code. This requirement
// arises because templated code gets compiled at each call site. If a call site is in a different module
// (i.e., outside of SpCore), and it references a static variable, then the module will get its own local
// copy of the static variable. This behavior only happens on Clang, because MSVC has a different methodology
// for handling static variables in shared libraries. See the link below for details:
//     https://stackoverflow.com/questions/31495877/i-receive-different-results-on-unix-and-win-when-use-static-members-with-static
//

FuncRegistry<USubsystem*, UWorld*, UClass*> g_get_subsystem_by_class_func_registry;
FuncRegistry<UClass*>                       g_get_static_class_func_registry;

USubsystem* UnrealClassRegistry::getSubsystemByClass(const std::string& subsystem_provider_class_name, UWorld* world, UClass* subsystem_uclass)
{
    return g_get_subsystem_by_class_func_registry.call(subsystem_provider_class_name, world, subsystem_uclass);
}

UClass* UnrealClassRegistry::getStaticClass(const std::string& class_name)
{
    return g_get_static_class_func_registry.call(class_name);
}
