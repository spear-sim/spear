#pragma once

// Some operating-system–specific header files define nil as nullptr or NULL.
// RPCLIB depends on the msgpack-c library, and msgpack-c has a different
// definition of nil. This creates naming conflicts when we include
// 'rpc/msgpack.hpp'. However, if MSGPACK_DISABLE_LEGACY_NIL is defined, then
// msgpack-c does not define nil, and we avoid the conflicts.
#define MSGPACK_DISABLE_LEGACY_NIL

// RPCLIB and Unreal have different definitions for check macro. This creates
// naming conflicts below when we include 'rpc/msgpack.hpp'. To avoid this
// conflict, we undefine the check macro before we include 'rpc/msgpack.hpp',
// and then redefine check to have the same definition as Unreal defines it
// after we include 'rpc/msgpack.hpp'. We use push_macro(...) and pop_macro
// for this purpose.
#pragma push_macro("check")
#ifdef check
#undef check
#endif

// For the TEXT macro, we do something similar to the check macro. The TEXT macro
// is defined by Unreal, but is also defined in 'winnt.h', which is included by
// 'rpc/msgpack.hpp' on Windows.
#pragma push_macro("TEXT")
#ifdef TEXT
#undef TEXT
#endif

#include <rpc/config.h>
#include <rpc/msgpack.hpp>
#include <rpc/server.h>

#pragma pop_macro("TEXT")
#pragma pop_macro("check")

// On Windows, 'rpc/msgpack.hpp' includes 'winnt.h', which in turn defines
// InterlockedCompareExchange as _InterlockedCompareExchange. This macro
// definition creates a naming conflict with Unreal and causes the following
// error:
//
// error C2039: '_InterlockedCompareExchange': is not a member of 'FWindowsPlatformAtomics'
//
// To avoid this issue, we need to undefine InterlockedCompareExchange before
// including any Unreal header files.
#ifdef InterlockedCompareExchange
#undef InterlockedCompareExchange
#endif
