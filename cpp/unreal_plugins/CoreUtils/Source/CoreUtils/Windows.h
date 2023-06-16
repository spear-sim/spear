//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h>

// Unreal and Windows have different definitions for the TEXT macro, so save its state and then restore it.
// Yield macro is initially undefined, and
// Windows defines it (https://stackoverflow.com/questions/18909719/for-what-is-the-function-like-macro-yield-in-winbase-h-line-97),
// so we save the state before and then restore it.
#if BOOST_OS_WINDOWS
    #pragma push_macro("TEXT")
    #undef TEXT
    #pragma push_macro("Yield")
    #undef Yield

    #include <Windows/MinWindows.h>

    #pragma pop_macro("Yield")
    #pragma pop_macro("TEXT")
#endif
