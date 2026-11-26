//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

//
// This header participates in SP_PLATFORM_PURE builds.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_WINDOWS

#include "SpCore/Platform.h"

// Inside Unreal, we include <windows.h> via Unreal's wrapper header, which prevents conflicts between the macros
// defined in <windows.h> and Unreal's own code. Outside of Unreal, we include <windows.h> directly.

#if BOOST_OS_WINDOWS
    #if SP_PLATFORM_UNREAL
        #include <Windows/WindowsHWrapper.h>
    #elif SP_PLATFORM_PURE
        #ifndef WIN32_LEAN_AND_MEAN
            #define WIN32_LEAN_AND_MEAN
        #endif
        #ifndef NOMINMAX
            #define NOMINMAX
        #endif
        #include <windows.h>
    #endif
#endif
