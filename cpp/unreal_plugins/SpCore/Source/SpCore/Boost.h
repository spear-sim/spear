//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

//
// This header participates in SP_PLATFORM_PURE builds.
//

#pragma once

//
// All headers that are required by this header go here.
//

#include <boost/predef.h> // BOOST_OS_LINUX, BOOST_OS_MACOS, BOOST_OS_WINDOWS

#include "SpCore/Platform.h"
#include "SpCore/SuppressCompilerWarnings.h"
#include "SpCore/Windows.h"

//
// All headers that are included as a convenience to users of this header go here. If a group of headers
// requires special handling, we place it into its own section below.
//

#pragma push_macro("check")  // Unreal macro, conflicts with Boost
#pragma push_macro("verify") // Unreal macro, conflicts with Boost

#undef check
#undef verify

// ------------------------------------------------------------------------------------------------------

// Avoid errors related to macros that are defined on Windows but conflict with <boost/asio.hpp>.

#pragma push_macro("PF_MAX")
#undef PF_MAX

#if BOOST_OS_WINDOWS
    #pragma push_macro("TRUE")
    #pragma push_macro("FALSE")

    #pragma push_macro("InterlockedCompareExchange")
    #pragma push_macro("InterlockedCompareExchangePointer")
    #pragma push_macro("InterlockedDecrement")
    #pragma push_macro("InterlockedExchange")
    #pragma push_macro("InterlockedExchangeAdd")
    #pragma push_macro("InterlockedIncrement")
    
    #undef TRUE
    #undef FALSE

    #undef InterlockedDecrement
    #undef InterlockedExchange
    #undef InterlockedExchangeAdd
    #undef InterlockedIncrement
    #undef InterlockedCompareExchange
    #undef InterlockedCompareExchangePointer

    #define TRUE true
    #define FALSE false

    #define InterlockedCompareExchange _InterlockedCompareExchange
    #define InterlockedCompareExchangePointer _InterlockedCompareExchangePointer
    #define InterlockedDecrement _InterlockedDecrement
    #define InterlockedExchange _InterlockedExchange
    #define InterlockedExchangeAdd _InterlockedExchangeAdd
    #define InterlockedIncrement _InterlockedIncrement
#endif

SP_BEGIN_SUPPRESS_COMPILER_WARNINGS
    #include <boost/asio.hpp>
SP_END_SUPPRESS_COMPILER_WARNINGS

#if BOOST_OS_WINDOWS
    #pragma pop_macro("TRUE")
    #pragma pop_macro("FALSE")

    #pragma pop_macro("InterlockedCompareExchange")
    #pragma pop_macro("InterlockedCompareExchangePointer")
    #pragma pop_macro("InterlockedDecrement")
    #pragma pop_macro("InterlockedExchange")
    #pragma pop_macro("InterlockedExchangeAdd")
    #pragma pop_macro("InterlockedIncrement")
#endif

#pragma pop_macro("PF_MAX")

// ----------------------------------------------------------------------------------------------------------

#include <boost/algorithm/string/case_conv.hpp> // boost::algorithm::to_lower_copy
#include <boost/algorithm/string/join.hpp>      // boost::algorithm::join

// ----------------------------------------------------------------------------------------------------------

#if BOOST_OS_WINDOWS || BOOST_OS_LINUX
    #define MAC_OS_X_VERSION_MIN_REQUIRED 0
#endif

#include <boost/align/aligned_allocator.hpp>

// ----------------------------------------------------------------------------------------------------------

#include <boost/callable_traits.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/current_function.hpp> // BOOST_CURRENT_FUNCTION
#include <boost/format.hpp>           // TODO: remove when we can use std::format on all platforms

// ----------------------------------------------------------------------------------------------------------

// macOS supports native posix shared memory objects, but doesn't export the _POSIX_SHARED_MEMORY_OBJECTS
// macro. However, Boost uses this macro to decide whether or not to use native posix shared memory objects,
// rather than emulating them through memory-mapped files. To address this issue, we explicitly define
// BOOST_INTERPROCESS_POSIX_SHARED_MEMORY_OBJECTS, because this forces Boost to use native posix shared
// memory objects. spear_ext includes this header too, so both sides of every interprocess object are
// guaranteed to use the same implementation.
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

    #include <boost/interprocess/exceptions.hpp>
    #include <boost/interprocess/mapped_region.hpp>
    #include <boost/interprocess/sync/file_lock.hpp>
SP_END_SUPPRESS_COMPILER_WARNINGS

// ----------------------------------------------------------------------------------------------------------

SP_BEGIN_SUPPRESS_COMPILER_WARNINGS
    #include <boost/lexical_cast.hpp>
SP_END_SUPPRESS_COMPILER_WARNINGS

// ----------------------------------------------------------------------------------------------------------

// Outside of Unreal, we use Boost.Stacktrace in header-only mode to print the call stack when an assert fails. Inside
// Unreal, we use FPlatformStackWalk instead, and we don't include Boost.Stacktrace at all, because on Windows it would
// add a link dependency on Dbgeng.lib and ole32.lib to every module that includes this header. Boost.Stacktrace
// requires _GNU_SOURCE or BOOST_STACKTRACE_GNU_SOURCE_NOT_REQUIRED to be defined on macOS and Linux, even though
// _Unwind_Backtrace(...) is available on all of the platforms we support.
#if SP_PLATFORM_PURE
    #define BOOST_STACKTRACE_GNU_SOURCE_NOT_REQUIRED

    SP_BEGIN_SUPPRESS_COMPILER_WARNINGS
        #include <boost/stacktrace.hpp> // boost::stacktrace::stacktrace, boost::stacktrace::to_string
    SP_END_SUPPRESS_COMPILER_WARNINGS
#endif

// ----------------------------------------------------------------------------------------------------------

#include <boost/test/debug.hpp>           // boost::debug::under_debugger
#include <boost/tokenizer.hpp>            // boost::char_separator
#include <boost/range/adaptor/map.hpp>    // boost::adaptors::map_keys
#include <boost/range/algorithm/copy.hpp> // boost::copy

#pragma pop_macro("check")
#pragma pop_macro("verify")
