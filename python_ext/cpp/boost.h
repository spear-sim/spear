//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_LINUX, BOOST_OS_MACOS, BOOST_OS_WINDOWS

// macOS supports native posix shared memory objects, but doesn't export the _POSIX_SHARED_MEMORY_OBJECTS
// macro. However, Boost uses this macro to decide whether or not to use native posix shared memory objects,
// rather than emulating them through memory-mapped files. To address this issue, we explicitly define
// BOOST_INTERPROCESS_POSIX_SHARED_MEMORY_OBJECTS, because this forces Boost to use native posix shared
// memory objects. This configuration must match SpCore/Boost.h exactly, because otherwise spear_ext and
// the Unreal application might silently use incompatible implementations of the same interprocess object.
#if BOOST_OS_MACOS
    #define BOOST_INTERPROCESS_POSIX_SHARED_MEMORY_OBJECTS
#endif

#if BOOST_OS_WINDOWS
    #include <boost/interprocess/windows_shared_memory.hpp>
#elif BOOST_OS_MACOS || BOOST_OS_LINUX
    #include <boost/interprocess/shared_memory_object.hpp>
#else
    #error
#endif

#include <boost/interprocess/mapped_region.hpp>
