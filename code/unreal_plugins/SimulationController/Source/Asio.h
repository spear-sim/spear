#pragma once

// We need to define InterlockedCompareExchange for the following reason.
// On Windows, 'asio.hpp' uses InterlockedCompareExchange macro.
// When 'asio.hpp' is included as below, if InterlockedCompareExchange macro is
// not defined, we get the following error -
// asio/detail/impl/win_iocp_io_context.ipp(351) : error C2039 :
// 'InterlockedCompareExchange' : is not a member of '`global namespace''.
// Hence, to avoid this error during 'asio.hpp' include, we define
// InterlockedCompareExchange macro as below.
#ifndef InterlockedCompareExchange
#define InterlockedCompareExchange _InterlockedCompareExchange
#endif

#include "IgnoreCompilerWarnings.h"
ENABLE_IGNORE_COMPILER_WARNINGS
#include <asio.hpp>
DISABLE_IGNORE_COMPILER_WARNINGS

// We need to undefine InterlockedCompareExchange for the following reason.
// InterlockedCompareExchange macro was defined above to avoid errors with
// 'asio.hpp' include. However, this macro definition creates a naming conflict
// with Unreal's code - FWindowsPlatformAtomics::InterlockedCompareExchange(...)
// - causing the following error:
//      error C2039: '_InterlockedCompareExchange': is not a member of
//      'FWindowsPlatformAtomics'.
// To avoid this issue, we need to undefine InterlockedCompareExchange before
// including any Unreal header files.
#ifdef InterlockedCompareExchange
#undef InterlockedCompareExchange
#endif
