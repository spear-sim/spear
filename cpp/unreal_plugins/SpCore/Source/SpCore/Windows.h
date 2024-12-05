//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h> // BOOST_OS_WINDOWS

#if BOOST_OS_WINDOWS
    #pragma push_macro("OPTIONAL") // Defined by <windows.h>, conflicts with our code
    #undef OPTIONAL

    #include <Windows/WindowsHWrapper.h>

    #pragma pop_macro("OPTIONAL")
#endif
