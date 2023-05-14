//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h>

#include "CoreUtils/SuppressCompilerWarnings.h"

// The Unreal check and verify macros conflict with Boost.
#pragma push_macro("check")
#pragma push_macro("verify")
#undef check
#undef verify

// macOS supports native posix shared memory objects, but doesn't export the _POSIX_SHARED_MEMORY_OBJECTS
// macro, but Boost uses this macro to decide whether or not to use native posix shared memory objects,
// rather than emulating them through memory-mapped files. To address this issue, we explicitly define
// BOOST_INTERPROCESS_POSIX_SHARED_MEMORY_OBJECTS, because this forces Boost to use native posix shared
// memory objects.
#if BOOST_OS_MACOS
    #define BOOST_INTERPROCESS_POSIX_SHARED_MEMORY_OBJECTS
#endif

SP_BEGIN_SUPPRESS_COMPILER_WARNINGS
#if BOOST_OS_WINDOWS
    #include <boost/interprocess/windows_shared_memory.hpp>
#elif BOOST_OS_MACOS || BOOST_OS_LINUX
    #include <boost/interprocess/shared_memory_object.hpp>
#else
    #error
#endif

#include <boost/interprocess/mapped_region.hpp>
SP_END_SUPPRESS_COMPILER_WARNINGS

// Restore the state of Unreal macros.
#pragma pop_macro("verify")
#pragma pop_macro("check")
