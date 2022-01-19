#pragma once

// The macro definitions in this file is coupled with RpcInclude.h
// Certain macros defined and undefined in RpcInclude.h are modified here for
// asio compatibility

#ifndef InterlockedCompareExchange
#define InterlockedCompareExchange _InterlockedCompareExchange
#endif

#include "Utils/IgnoreCompilerWarnings.h"

ENABLE_IGNORE_COMPILER_WARNINGS
#include "asio.hpp"
DISABLE_IGNORE_COMPILER_WARNINGS

#ifdef InterlockedCompareExchange
#undef InterlockedCompareExchange
#endif
