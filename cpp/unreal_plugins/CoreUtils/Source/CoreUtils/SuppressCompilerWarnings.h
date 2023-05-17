//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) Microsoft Corporation. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <boost/predef.h>

#if BOOST_COMP_MSVC
    // keep these warnings sorted
    #define SP_BEGIN_SUPPRESS_COMPILER_WARNINGS __pragma(warning(push)) __pragma(warning(disable : 4005 4100 4189 4191 4244 4245 4239 4267 4365 4456 4464 4505 4514 4571 4624 4625 4626 4668 4701 4710 4820 4917 4996 5026 5027 5031))
    #define SP_END_SUPPRESS_COMPILER_WARNINGS __pragma(warning(pop))

#elif BOOST_COMP_CLANG
    // keep these warnings sorted
    #define SP_BEGIN_SUPPRESS_COMPILER_WARNINGS \
        _Pragma("clang diagnostic push")                                     \
        _Pragma("clang diagnostic ignored \"-Wcast-qual\"")                  \
        _Pragma("clang diagnostic ignored \"-Wctor-dtor-privacy\"")          \
        _Pragma("clang diagnostic ignored \"-Wdelete-non-virtual-dtor\"")    \
        _Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"")    \
        _Pragma("clang diagnostic ignored \"-Wmissing-field-initializers\"") \
        _Pragma("clang diagnostic ignored \"-Wold-style-cast\"")             \
        _Pragma("clang diagnostic ignored \"-Wredundant-decls\"")            \
        _Pragma("clang diagnostic ignored \"-Wreturn-type\"")                \
        _Pragma("clang diagnostic ignored \"-Wshadow\"")                     \
        _Pragma("clang diagnostic ignored \"-Wstrict-overflow\"")            \
        _Pragma("clang diagnostic ignored \"-Wswitch-default\"")             \
        _Pragma("clang diagnostic ignored \"-Wundef\"")                      \
        _Pragma("clang diagnostic ignored \"-Wunused-variable\"")            \
        _Pragma("clang diagnostic ignored \"-Wunused-parameter\"")

        //
        // Need /**/ style comments below
        //

        /*
        _Pragma("clang diagnostic ignored \"-Werror\"")                      \
        _Pragma("clang diagnostic ignored \"-Werror=\"")                     \
        _Pragma("clang diagnostic ignored \"-Wformat=\"")                    \
        _Pragma("clang diagnostic ignored \"-Wpedantic\"")                   \
        _Pragma("clang diagnostic ignored \"-Wunused-variable\"")            \
        */

    #define SP_END_SUPPRESS_COMPILER_WARNINGS \
        _Pragma("clang diagnostic pop")

#else
    #error
#endif

// TODO: remove these #defines when we're ready to completely switch to SP_BEGIN_SUPPRESS_COMPILER_WARNINGS and SP_END_SUPPRESS_COMPILER_WARNINGS
#define BEGIN_SUPPRESS_COMPILER_WARNINGS SP_BEGIN_SUPPRESS_COMPILER_WARNINGS
#define END_SUPPRESS_COMPILER_WARNINGS SP_END_SUPPRESS_COMPILER_WARNINGS
