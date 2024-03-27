//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_WINDOWS

#if BOOST_OS_WINDOWS
    #pragma push_macro("OPTIONAL") // Defined by <windows.h>, conflicts with our code
    #pragma push_macro("TEXT")     // Unreal macro, conflicts with <windows.h>    
    #pragma push_macro("Yield")    // Defined by <windows.h>, conflicts with Unreal (e.g., FPlatformProcess::Yield)
    #undef OPTIONAL
    #undef TEXT
    #undef Yield

    #include <Windows/MinWindows.h>

    #pragma pop_macro("Yield")
    #pragma pop_macro("TEXT")
    #pragma pop_macro("OPTIONAL")
#endif
