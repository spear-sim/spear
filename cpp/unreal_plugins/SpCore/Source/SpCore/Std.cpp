//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/Std.h"

#include <stdarg.h> // va_end, va_list, va_start
#include <stdio.h>  // vsscanf
#include <stdlib.h> // strtol

#include <boost/predef.h> // BOOST_OS_LINUX

#if BOOST_OS_LINUX
    SP_EXTERN_C long int __isoc23_sscanf(const char *str, const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        int result = vsscanf(str, format, args);
        va_end(args);
        return result;
    }

    SP_EXTERN_C long int __isoc23_strtol(const char* str, char** endptr, int base)
    {
        return strtol(str, endptr, base);
    }
#endif
