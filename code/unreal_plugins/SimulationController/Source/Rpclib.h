#pragma once

// Unreal and Windows have different definitions for the TEXT macro, so save its state.
#pragma push_macro("TEXT")
#ifdef TEXT
#undef TEXT
#endif

// Unreal and rpclib have different definitions for the check macro, so we save its state.
#pragma push_macro("check")
#ifdef check
#undef check
#endif

// If we have already included Windows.h in a different header, then we must have already
// invoked UNDEFINE_WINDOWS_HEADER_MACROS from that header. In this case, we must invoke
// DEFINE_WINDOWS_HEADER_MACROS, since including Windows.h again from here will have no
// effect. If we have not already included Windows.h, then include it here for the first
// time.
#ifdef _MSC_VER
    #ifdef _WINDOWS_
        #include "WindowsHeaderMacrosDefine.h"
    #else
        #include <Windows/MinWindowsH.h>
    #endif
#endif

// Some operating-system–specific header files define nil as nullptr or NULL. rpclib depends
// on the msgpack-c library, and msgpack-c has a different definition of nil. This creates
// naming conflicts when we include 'rpc/msgpack.hpp'. However, if MSGPACK_DISABLE_LEGACY_NIL
// is defined, then msgpack-c does not define nil, and we avoid the conflicts.
#define MSGPACK_DISABLE_LEGACY_NIL

// Include rpclib headers
#include <rpc/config.h>
#include <rpc/msgpack.hpp>
#include <rpc/server.h>

// Undefine Windows.h macros to prevent them from polluting the global namespace. Any other
// header files that are included after this, and depend on Windows.h macros, must invoke
// WindowsHeaderMacrosDefine.h, since including Windows.h again will have no effect.
#ifdef _MSC_VER
    #include "WindowsHeaderMacrosUndefine.h"
#endif

// Restore the state of Unreal macros.
#pragma pop_macro("check")
#pragma pop_macro("TEXT")
