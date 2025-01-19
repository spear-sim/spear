//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_MACOS, BOOST_OS_WINDOWS

#include "SpCore/Windows.h" // On Windows, rpclib expects <windows.h> to have been included

#pragma push_macro("check") // Unreal macro, conflicts with rpclib
#undef check

// rpclib unconditionally checks the macros below, which are undefined on Windows, leading to warnings. To
// avoid these warnings, we define the macros as 0, which preserves rpclib's intended macro checking logic.
#if BOOST_OS_WINDOWS
    #define MSGPACK_ARCH_AMD64 0
    #define __GNUC__ 0
#endif

// Some macOS-specifc header files define nil as nullptr or NULL. But rpclib depends on the msgpack-c library,
// which has a different definition of nil. This creates naming conflicts when we include <rpc/msgpack.hpp>.
// However, if MSGPACK_DISABLE_LEGACY_NIL is defined, then msgpack-c does not define nil, and we avoid the
// conflicts.
#if BOOST_OS_MACOS
    #define MSGPACK_DISABLE_LEGACY_NIL
#endif

#include <rpc/config.h>
#include <rpc/msgpack.hpp> // clmdep_msgpack, MSGPACK_ADD_ENUM
#include <rpc/server.h>

#if BOOST_OS_WINDOWS
    #undef MSGPACK_ARCH_AMD64
    #undef __GNUC__
#endif

#if BOOST_OS_MACOS
    #undef MSGPACK_DISABLE_LEGACY_NIL
#endif

#pragma pop_macro("check")
