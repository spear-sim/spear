//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h>

// Unreal and Windows have different definitions for the TEXT macro, so we save it, include Windows.h, and
// then restore it. Windows also defines a Yield macro, which can interfere with various Unreal functions
// (e.g., FPlatformProcess::Yield), so we use the same approach (of calling push_macro() and pop_macro())
// to limit the scope of Windows' Yield macro to remain within Windows.h. See the following link for details:
//     https://stackoverflow.com/questions/18909719/for-what-is-the-function-like-macro-yield-in-winbase-h-line-97
#if BOOST_OS_WINDOWS
    #pragma push_macro("TEXT")
    #pragma push_macro("Yield")
    #undef TEXT
    #undef Yield

    #include <Windows/MinWindows.h>

    #pragma pop_macro("Yield")
    #pragma pop_macro("TEXT")
#endif
