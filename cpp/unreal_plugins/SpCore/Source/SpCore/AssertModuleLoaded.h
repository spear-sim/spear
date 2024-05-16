//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <filesystem>
#include <iostream> // std::cout

#include <GenericPlatform/GenericPlatformMisc.h>
#include <HAL/Platform.h> // TEXT

#include "SpCore/Boost.h"

// TODO: remove platform-specific include
#if BOOST_COMP_MSVC
    #include <format>
#endif

// We need a special macro to check if the SpCore module is loaded that avoids all SpCore functions, because
// if we attempt to use an SpCore function before it is loaded, it will result in undefined behavior.

#if BOOST_COMP_MSVC
    #define SP_ASSERT_MODULE_LOADED_IMPL(module_name, current_file, current_line) \
        if (!FModuleManager::Get().IsModuleLoaded(FName(TEXT(module_name)))) {                                                             \
            std::cout <<                                                                                                                   \
                "[SPEAR | " + std::filesystem::path(current_file).filename().string() + ":" + std::format("{:04}", current_line) + "] " << \
                "ERROR: \"" << module_name << "\" module not loaded, terminating..." << std::endl;                                         \
            bool force = true;                                                                                                             \
            FGenericPlatformMisc::RequestExit(force);                                                                                      \
        }
#elif BOOST_COMP_CLANG
    #define SP_ASSERT_MODULE_LOADED_IMPL(module_name, current_file, current_line) \
        if (!FModuleManager::Get().IsModuleLoaded(FName(TEXT(module_name)))) {                                                                     \
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
