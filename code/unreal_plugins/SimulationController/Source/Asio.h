#pragma once

#include "CompilerWarningUtils.h"

// Unreal and Windows have different definitions for the TEXT macro, so save its state.
#pragma push_macro("TEXT")
#ifdef TEXT
#undef TEXT
#endif

// If we have already included Windows.h in a different header, then we must have already
// invoked WindowsHeaderMacrosUndefine.h from that header. In this case, we must invoke
// WindowsHeaderMacrosDefine.h, since including Windows.h again from here will have no
// effect. If we have not already included Windows.h, then include it here for the first
// time.
#ifdef _MSC_VER
    #ifdef _WINDOWS_
        #include "WindowsHeaderMacrosDefine.h"
    #else
        #include <Windows/MinWindowsH.h>
    #endif
#endif

// Include asio headers.
BEGIN_IGNORE_COMPILER_WARNINGS
#include <asio.hpp>
END_IGNORE_COMPILER_WARNINGS

// Undefine Windows.h macros to prevent them from polluting the global namespace. Any other
// header files that are included after this, and depend on Windows.h macros, must invoke
// WindowsHeaderMacrosDefine.h, since including Windows.h again will have no effect.
#ifdef _MSC_VER
    #include "WindowsHeaderMacrosUndefine.h"
#endif

// Restore the state of Unreal macros.
#pragma pop_macro("TEXT")
