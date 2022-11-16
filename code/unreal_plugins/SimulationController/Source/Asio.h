#pragma once

#include "CompilerWarningUtils.h"

// Unreal and Windows have different definitions for the TEXT macro, so save its state.
#ifdef _MSC_VER
    #pragma push_macro("TEXT")
    #undef TEXT
#endif

#ifdef _MSC_VER
    #include <Windows/MinWindows.h>
#endif

// Include asio headers.
BEGIN_IGNORE_COMPILER_WARNINGS
#include <asio.hpp>
END_IGNORE_COMPILER_WARNINGS

// Restore the state of the TEXT macro.
#ifdef _MSC_VER
    #pragma pop_macro("TEXT")
#endif