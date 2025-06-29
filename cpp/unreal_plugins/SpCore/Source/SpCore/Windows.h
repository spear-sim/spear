//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

// This file is included in SpCore/Boost.h so we can't include SpCore/Boost.h to get boost/predef.h
#include <boost/predef.h> // BOOST_OS_WINDOWS

#if BOOST_OS_WINDOWS
    #pragma push_macro("OPTIONAL") // Defined by <windows.h>, conflicts with our code
    #undef OPTIONAL

    #include <Windows/WindowsHWrapper.h>

    #pragma pop_macro("OPTIONAL")
#endif
