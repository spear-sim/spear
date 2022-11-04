#pragma once

// Some operating system specific header files have defined nil as nullptr or
// NULL. RPCLIB depends on msgpack-c library, and msgpack-c has a different
// definition of nil. This creates naming conflicts when we include
// 'rpc/msgpack.hpp' below. If MSGPACK_DISABLE_LEGACY_NIL is defined, msgpack-c
// does not redefine nil, avoiding the conflict. Hence, we define
// MSGPACK_DISABLE_LEGACY_NIL below.
#define MSGPACK_DISABLE_LEGACY_NIL

// Both RPCLIB library and Unreal have different definitions for check macro.
// This creates conflict below when we include 'rpc/msgpack.hpp'.
// To avoid this conflict, we need to undefine check macro before we include
// 'rpc/msgpack.hpp', and then redefine check to have the same definition as
// Unreal defines it. Hence, before we undefine check, we copy its definition so
// that it can be used later to redefine check. We use push_macro() to copy and
// store check's definition in a stack.
#pragma push_macro("check")
#ifdef check
#undef check
#endif

// For the TEXT macro, we do something similar to the check macro. The TEXT macro
// is defined in 'winnt.h', which is included by 'rpc/msgpack.hpp' on Windows.
#pragma push_macro("TEXT")
#ifdef TEXT
#undef TEXT
#endif

#include <rpc/config.h>
#include <rpc/msgpack.hpp>
#include <rpc/server.h>

#pragma pop_macro("TEXT")
#pragma pop_macro("check")

// We need to undefine InterlockedCompareExchange for the following reason.
// 'rpc/msgpack.hpp' includes 'winnt.h' when compiled on Windows, which in turn
// defines InterlockedCompareExchange as _InterlockedCompareExchange. This macro
// definition creates a naming conflict with Unreal's code -
// FWindowsPlatformAtomics::InterlockedCompareExchange(...) - causing the
// following error:
//      error C2039: '_InterlockedCompareExchange': is not a member of
//      'FWindowsPlatformAtomics'.
// To avoid this issue, we need to undefine InterlockedCompareExchange before
// including any Unreal header files.
#ifdef InterlockedCompareExchange
#undef InterlockedCompareExchange
#endif
