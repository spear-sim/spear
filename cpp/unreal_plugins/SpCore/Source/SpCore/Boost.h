//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_MACOS, BOOST_OS_LINUX, BOOST_OS_WINDOWS

#include "SpCore/SuppressCompilerWarnings.h"
#include "SpCore/Windows.h"

#pragma push_macro("check")  // Unreal macro, conflicts with Boost
#pragma push_macro("verify") // Unreal macro, conflicts with Boost
#undef check
#undef verify

#include <boost/circular_buffer.hpp>
#include <boost/current_function.hpp> // BOOST_CURRENT_FUNCTION

SP_BEGIN_SUPPRESS_COMPILER_WARNINGS
    #include <boost/lexical_cast.hpp>
SP_END_SUPPRESS_COMPILER_WARNINGS

#include <boost/tokenizer.hpp> // boost::char_separator

//
// boost::algorithm
//

#include <boost/algorithm/string/case_conv.hpp> // boost::algorithm::to_lower_copy

//
// boost::asio
//

// On Windows, the <boost/asio.hpp> header eventually includes <winsock2.h>, which expects an OPTIONAL
// macro to be defined. Under normal circumstances, this macro would have been defined by another Windows
// header by the time <winsock2.h> uses it. But this macro interferes with our code, so we undefine it at
// the end of "SpCore/Windows.h". And since headers are only expanded once per compilation unit, OPTIONAL
// will not be redefined when <winsock2.h> includes the header that would normally define it. Therefore,
// our strategy is to redefine it here, include <boost/asio.hpp>, and then undefine it.
#if BOOST_OS_WINDOWS
    #pragma push_macro("OPTIONAL")
    #undef OPTIONAL
    #define OPTIONAL
#endif

SP_BEGIN_SUPPRESS_COMPILER_WARNINGS
    #include <boost/asio.hpp>
SP_END_SUPPRESS_COMPILER_WARNINGS

#if BOOST_OS_WINDOWS
    #pragma pop_macro("OPTIONAL")
#endif

//
// boost::interprocess
//

// macOS supports native posix shared memory objects, but doesn't export the _POSIX_SHARED_MEMORY_OBJECTS
// macro. However, Boost uses this macro to decide whether or not to use native posix shared memory objects,
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

//
// boost::range
//

#include <boost/range/adaptor/map.hpp>    // boost::adaptors::map_keys
#include <boost/range/algorithm/copy.hpp> // boost::copy

#pragma pop_macro("verify")
#pragma pop_macro("check")
