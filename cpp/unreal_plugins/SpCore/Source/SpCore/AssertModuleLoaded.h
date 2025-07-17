//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <filesystem>
#include <iostream> // std::cout

#include <boost/predef.h> // BOOST_COMP_MSVC, BOOST_COMP_CLANG

#include <GenericPlatform/GenericPlatformMisc.h>
#include <HAL/Platform.h> // TEXT
#include <Modules/ModuleManager.h>

// TODO: remove platform-specific include
#if BOOST_COMP_MSVC
    #include <format>
#endif

// Our usual SP_ASSERT macro uses some helper functions that are defined in the SpCore module. So if we tried
// to use SP_ASSERT to confirm that SpCore was loaded, but it wasn't loaded, we would be invoking undefined
// behavior. We therefore need a special assert macro that avoids all module functions.

#if BOOST_COMP_MSVC
    #define SP_ASSERT_MODULE_LOADED_IMPL(module_name, current_file, current_line) \
        if (!FModuleManager::Get().IsModuleLoaded(module_name)) {                                                                          \
            std::cout <<                                                                                                                   \
                "[SPEAR | " + std::filesystem::path(current_file).filename().string() + ":" + std::format("{:04}", current_line) + "] " << \
                "ERROR: \"" << module_name << "\" module not loaded, terminating..." << std::endl;                                         \
            bool force = true;                                                                                                             \
            FGenericPlatformMisc::RequestExit(force);                                                                                      \
        }
#elif BOOST_COMP_CLANG
    #define SP_ASSERT_MODULE_LOADED_IMPL(module_name, current_file, current_line) \
        if (!FModuleManager::Get().IsModuleLoaded(module_name)) {                                                                                  \
            std::cout <<                                                                                                                           \
                "[SPEAR | " + std::filesystem::path(current_file).filename().string() + ":" + (boost::format("%04d")%current_line).str() + "] " << \
                "ERROR: \"" << module_name << "\" module not loaded, terminating..." << std::endl;                                                 \
            bool force = true;                                                                                                                     \
            FGenericPlatformMisc::RequestExit(force);                                                                                              \
        }
#else
    #error
#endif

#define SP_ASSERT_MODULE_LOADED(module_name) \
    SP_ASSERT_MODULE_LOADED_IMPL(module_name, __FILE__, __LINE__)
