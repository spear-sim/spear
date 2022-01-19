#pragma once

#include "IgnoreCompilerWarnings.h"

ENABLE_IGNORE_COMPILER_WARNINGS

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#ifdef nil
#undef nil
#endif
#undef FLOAT
#undef check
#pragma once

#include "rpc/msgpack.hpp"

// Need this to avoid this
//     error C2039: '_InterlockedCompareExchange': is not a member of
//     'FWindowsPlatformAtomics'
// caused due to rpc/msgpack.hpp which includes 'winnt.h' which sets #define
// InterlockedCompareExchange _InterlockedCompareExchange which interferes with
// Unreal's code: FWindowsPlatformAtomics::InterlockedCompareExchange(...)

#ifdef InterlockedCompareExchange
#undef InterlockedCompareExchange
#endif

DISABLE_IGNORE_COMPILER_WARNINGS

#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
