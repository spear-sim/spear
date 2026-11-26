//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

//
// This header participates in SP_PLATFORM_PURE builds.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_LINUX

// We include this header instead of including <HAL/Platform.h> directly in headers that are shared with code that
// is built outside of Unreal (e.g., python_ext). SP_PLATFORM_UNREAL is 1 when building inside Unreal, and SP_PLATFORM_PURE
// is 1 when building outside of Unreal. UnrealBuildTool defines __UNREAL__ for every module it builds. We use
// defined(...) here because __UNREAL__ isn't defined outside of Unreal, which would trigger -Wundef warnings.

#if defined(__UNREAL__)
    #define SP_PLATFORM_UNREAL 1
    #define SP_PLATFORM_PURE 0
#else
    #define SP_PLATFORM_UNREAL 0
    #define SP_PLATFORM_PURE 1
#endif

// Outside of Unreal, we define the basic integer types that Unreal defines in <HAL/Platform.h>, so code that is shared
// with python_ext can use them in either type of build.

#if SP_PLATFORM_PURE
    #include <stdint.h> // int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t

    using int8   = int8_t;
    using int16  = int16_t;
    using int32  = int32_t;
    using int64  = int64_t;
    using uint8  = uint8_t;
    using uint16 = uint16_t;
    using uint32 = uint32_t;
    using uint64 = uint64_t;
#endif

// Outside of Unreal, code that includes our headers is responsible for defining plugin _API macros (e.g., SPCORE_API).
// We don't define them here, because this header shouldn't need to know about the plugins that depend on SpCore.

#if SP_PLATFORM_UNREAL
    #include <HAL/Platform.h>          // int8, int16, int32, int64, uint8, uint16, uint32, uint64, SPCORE_API and other plugin _API macros
    #include <HAL/PlatformMisc.h>      // FPlatformMisc
    #include <HAL/PlatformStackWalk.h> // FPlatformStackWalk
#endif

// On Linux, our third-party libraries can reference __isoc23_sscanf(...) and __isoc23_strtol(...) if they were built
// against glibc 2.38 or later, but the glibc in Unreal's Linux toolchain doesn't provide these functions. So we provide
// them ourselves in Platform.cpp. This isn't necessary outside of Unreal, where we link against the system glibc.

#if SP_PLATFORM_UNREAL
    #if BOOST_OS_LINUX
        #ifdef __cplusplus
            #define SP_EXTERN_C extern "C"
        #else
            #define SP_EXTERN_C
        #endif
        SP_EXTERN_C SPCORE_API long int __isoc23_sscanf(const char *str, const char *format, ...);
        SP_EXTERN_C SPCORE_API long int __isoc23_strtol(const char* str, char** endptr, int base);
    #endif
#endif
