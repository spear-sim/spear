//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_WINDOWS

#if BOOST_OS_WINDOWS
    #include <Windows/WindowsHWrapper.h>

    // The Windows SDK's <winsock.h> defines a PF_MAX macro (the maximum protocol family value) that collides with
    // EPixelFormat::PF_MAX from the RHI headers. We force <winsock2.h> to be included here, which sets _WINSOCKAPI_ and
    // thereby prevents <winsock.h> (and its PF_MAX macro) from being pulled in later (e.g., transitively through the
    // engine's networking headers). The #undef below covers the case where <winsock.h> was already included before
    // this header. Include this header first whenever downstream code needs to refer to EPixelFormat::PF_MAX.
    #include "Windows/AllowWindowsPlatformTypes.h"
        #include <winsock2.h>
    #include "Windows/HideWindowsPlatformTypes.h"

    #ifdef PF_MAX
        #undef PF_MAX
    #endif
#endif
