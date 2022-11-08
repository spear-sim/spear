#pragma once

// On Windows, 'asio.hpp' uses the InterlockedCompareExchange macro. When
// 'asio.hpp' is included as below, if InterlockedCompareExchange macro
// is not defined, we get the following error:
//
// error C2039 : 'InterlockedCompareExchange' : is not a member of 'global namespace
//
// To avoid this error when including 'asio.hpp', we define
// InterlockedCompareExchange macro as below.
#ifndef InterlockedCompareExchange
#define InterlockedCompareExchange _InterlockedCompareExchange
#endif

#include "IgnoreCompilerWarnings.h"
BEGIN_IGNORE_COMPILER_WARNINGS
#include <asio.hpp>
END_IGNORE_COMPILER_WARNINGS

// When the InterlockedCompareExchange macro is defined above, it creates a
// conflict with the Unreal function FWindowsPlatformAtomics::InterlockedCompareExchange(...).
// This causes the following error:
//
// error C2039: '_InterlockedCompareExchange': is not a member of 'FWindowsPlatformAtomics'
//
// To avoid this error, we undefine InterlockedCompareExchange before
// including any Unreal header files.
#ifdef InterlockedCompareExchange
#undef InterlockedCompareExchange
#endif
