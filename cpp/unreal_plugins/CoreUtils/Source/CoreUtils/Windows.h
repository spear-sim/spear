//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h>

// Unreal and Windows have different definitions for the TEXT macro, so save its state and then restore it.
#if BOOST_OS_WINDOWS
    #pragma push_macro("TEXT")
    #undef TEXT

    #include <Windows/MinWindows.h>

    #pragma pop_macro("TEXT")
#endif
