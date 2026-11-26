//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

//
// This header participates in SP_PLATFORM_PURE builds.
//

#pragma once

#include "SpCore/Platform.h"

// SpServices code that is shared with python_ext is used on both sides of an RPC connection, so we define
// SP_PLATFORM_SERVER and SP_PLATFORM_CLIENT to describe which side of the connection we're building. The server
// always runs inside Unreal, and the client always runs outside of Unreal (e.g., in python_ext).

#if SP_PLATFORM_UNREAL
    #define SP_PLATFORM_SERVER 1
    #define SP_PLATFORM_CLIENT 0
#elif SP_PLATFORM_PURE
    #define SP_PLATFORM_SERVER 0
    #define SP_PLATFORM_CLIENT 1
#else
    #error
#endif
