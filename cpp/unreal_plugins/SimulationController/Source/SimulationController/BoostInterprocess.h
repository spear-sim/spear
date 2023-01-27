//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h>

// macOS supports native posix shared memory objects, but doesn't export the _POSIX_SHARED_MEMORY_OBJECTS
// macro, but Boost uses this macro to decide whether or not to use native posix shared memory objects,
// rather than emulating them through memory-mapped files. To address this issue, we explicitly define
// BOOST_INTERPROCESS_POSIX_SHARED_MEMORY_OBJECTS, because this forces Boost to use native posix shared
// memory objects.
#if BOOST_OS_MACOS
    #define BOOST_INTERPROCESS_POSIX_SHARED_MEMORY_OBJECTS
#endif

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
