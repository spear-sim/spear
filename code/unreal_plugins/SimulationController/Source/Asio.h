#pragma once

// Unreal and Windows have different definitions for the TEXT macro, so save its state
// and then restore it.
#ifdef _MSC_VER
    #pragma push_macro("TEXT")
    #undef TEXT

    #include <Windows/MinWindows.h>

    #pragma pop_macro("TEXT")
#endif

#include "CompilerWarningUtils.h"

BEGIN_IGNORE_COMPILER_WARNINGS
#include <asio.hpp>
END_IGNORE_COMPILER_WARNINGS
