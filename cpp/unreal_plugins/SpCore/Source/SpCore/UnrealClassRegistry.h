//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>

#include <HAL/Platform.h> // SPCORE_API

#include "SpCore/FuncRegistry.h"
#include "SpCore/Unreal.h"

class UClass;
class USubsystem;
class UWorld;

//
// Helper macros to avoid repetitive boilerplate code at call sites.
//

#define SP_REGISTER_SUBSYSTEM_PROVIDER_CLASS(subsystem_provider_class)   UnrealClassRegistry::registerSubsystemProviderClass<subsystem_provider_class>(#subsystem_provider_class)
#define SP_REGISTER_INTERFACE_CLASS(interface_class)                     UnrealClassRegistry::registerInterfaceClass<interface_class>(#interface_class)

#define SP_UNREGISTER_SUBSYSTEM_PROVIDER_CLASS(subsystem_provider_class) UnrealClassRegistry::unregisterSubsystemProviderClass<subsystem_provider_class>(#subsystem_provider_class)
#define SP_UNREGISTER_INTERFACE_CLASS(interface_class)                   UnrealClassRegistry::unregisterInterfaceClass<interface_class>(#interface_class)

//
// We need the variables below to be globals because they are referenced in templated code. This requirement
// arises because templated code gets compiled at each call site. If a call site is in a different module
// (i.e., outside of SpCore), and it references a static variable, then the module will get its own local
// copy of the static variable. This behavior only happens on macOS and Linux, because Windows has a
// different methodology for handling static variables in shared libraries. See the link below for details:
//     https://stackoverflow.com/questions/31495877/i-receive-different-results-on-unix-and-win-when-use-static-members-with-static
//

extern SPCORE_API FuncRegistry<USubsystem*, UWorld*, UClass*> g_get_subsystem_by_class_func_registry;
extern SPCORE_API FuncRegistry<UClass*>                       g_get_static_class_func_registry;

class SPCORE_API UnrealClassRegistry
{
public:
    UnrealClassRegistry() = delete;
    ~UnrealClassRegistry() = delete;

    static USubsystem* getSubsystemByClass(const std::string& class_name, UWorld* world, UClass* uclass);
    static UClass* getStaticClass(const std::string& class_name);

    template <CSubsystemProvider TSubsystemProvider>
    static void registerSubsystemProviderClass(const std::string& class_name)
    {
        g_get_subsystem_by_class_func_registry.registerFunc(
            class_name, [](UWorld* world, UClass* uclass) -> USubsystem* {
                return Unreal::getSubsystemByClass<TSubsystemProvider>(world, uclass);
            });
    }

    template <CSubsystemProvider TSubsystemProvider>
    static void unregisterSubsystemProviderClass(const std::string& class_name)
    {
        g_get_subsystem_by_class_func_registry.unregisterFunc(class_name);
    }

    template <CInterface TInterface>
    static void registerInterfaceClass(const std::string& class_name)
    {
        g_get_static_class_func_registry.registerFunc(
            class_name, []() -> UClass* {
                return Unreal::getStaticClass<TInterface>();
            });
    }

    template <CInterface TInterface>
    static void unregisterInterfaceClass(const std::string& class_name)
    {
        g_get_static_class_func_registry.unregisterFunc(class_name);
    }
};
