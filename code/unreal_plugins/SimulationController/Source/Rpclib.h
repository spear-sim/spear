#pragma once

// Some operating-system–specific header files define nil as nullptr or NULL.
// rpclib depends on the msgpack-c library, and msgpack-c has a different
// definition of nil. This creates naming conflicts when we include
// 'rpc/msgpack.hpp'. However, if MSGPACK_DISABLE_LEGACY_NIL is defined, then
// msgpack-c does not define nil, and we avoid the conflicts.
#define MSGPACK_DISABLE_LEGACY_NIL

// rpclib and Unreal have different definitions for the check macro. This
// creates naming conflicts when we include 'rpc/msgpack.hpp'. To avoid these
// conflicts, we undefine the check macro before we include 'rpc/msgpack.hpp',
// and then redefine check to have the same definition as Unreal defines it
// after we include 'rpc/msgpack.hpp'. We use push_macro(...) and pop_macro
// for this purpose.
#pragma push_macro("check")
#ifdef check
#undef check
#endif

// When compiling on Windows, rpclib will include Windows headers, which will define
// macros that conflict with Unreal Engine code. So we wrap our rpclib includes with
// PreWindowsApi.h and PostWindowsApi.h.
#ifdef _MSC_VER
#include <Windows/PreWindowsApi.h>
#endif

#include <rpc/config.h>
#include <rpc/msgpack.hpp>
#include <rpc/server.h>

#ifdef _MSC_VER
#include <Windows/PostWindowsApi.h>
#endif

#pragma pop_macro("check")
