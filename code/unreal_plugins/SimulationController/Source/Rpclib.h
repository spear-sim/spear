#pragma once

// Unreal and Windows have different definitions for the TEXT macro, so save its state
// and then restore it.
#ifdef _MSC_VER
    #pragma push_macro("TEXT")
    #undef TEXT

    #include <Windows/MinWindows.h>

    #pragma pop_macro("TEXT")
#endif

// Unreal and rpclib have different definitions for the check macro, so save its state.
#pragma push_macro("check")
#undef check

// Some macOS-specifc header files define nil as nullptr or NULL. But rpclib depends on the
// msgpack-c library, which has a different definition of nil. This creates naming conflicts
// when we include 'rpc/msgpack.hpp'. However, if MSGPACK_DISABLE_LEGACY_NIL is defined, then
// msgpack-c does not define nil, and we avoid the conflicts.
#define MSGPACK_DISABLE_LEGACY_NIL

// Include rpclib headers.
#include <rpc/config.h>
#include <rpc/msgpack.hpp>
#include <rpc/server.h>

// Restore the state of the check macro.
#pragma pop_macro("check")
