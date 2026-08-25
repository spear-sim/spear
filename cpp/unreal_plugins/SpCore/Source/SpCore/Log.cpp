//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/Log.h"

#include <filesystem>
#include <iostream> // std::cout
#include <mutex>    // std::recursive_mutex
#include <regex>
#include <string>   // std::string::operator<<
#include <vector>

#include <Containers/UnrealString.h> // FString
#include <CoreGlobals.h>             // IsRunningCommandlet
#include <Misc/CoreMisc.h>           // IsRunningGame
#include <HAL/Platform.h>            // TEXT
#include <Logging/LogMacros.h>       // DECLARE_LOG_CATEGORY_EXTERN, DEFINE_LOG_CATEGORY, UE_LOG
#include <Misc/CommandLine.h>
#include <Misc/Parse.h>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

// TODO: remove platform-specific include
#if BOOST_COMP_MSVC
    #include <format>
#elif BOOST_COMP_CLANG
    #include "SpCore/Boost.h"
#else
    #error
#endif

DEFINE_LOG_CATEGORY(LogSpear);

std::recursive_mutex g_mutex;

void Log::logCurrentFunction(const std::filesystem::path& current_file, int current_line, const std::string& current_function)
{
    log(current_file, current_line, getCurrentFunctionAbbreviated(current_function));
}

void Log::logString(const std::string& str)
{
    logStringToUnreal(str); // logStringToUnreal(...) is a no-op unless we're in editor mode via GUI
    logStringToStdout(str);
}

void Log::logStringToStdout(const std::string& str)
{
    std::lock_guard<std::recursive_mutex> lock(getStdoutMutex());
    std::cout << str << std::endl;
}

std::recursive_mutex& Log::getStdoutMutex()
{
    return g_mutex;
}

void Log::logStringToUnreal(const std::string& str)
{
    // This is the single source of truth for whether we should route a message to UE_LOG at all: only in
    // editor mode via GUI, not via the command line (e.g., during cooking or using -game), and not in
    // standalone mode. Assert.cpp's print() function also calls this function directly (rather than going
    // through logString(...)).
    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet() || IsRunningGame()) {
            return;
        }
    #else // standalone mode
        return;
    #endif

    // We need to use TEXT(...) and *Unreal::toFString(...) because if we use Unreal::toTCharPtr(...) we will
    // fail static assertions and get errors in the UE_LOG macro:
    //     error: static assertion failed due to requirement 'std::is_const_v<Unreal::TCharPtr>': Formatting string must be a const TCHAR array.
    //     error: static assertion failed due to requirement 'TIsArrayOrRefOfTypeByPredicate<Unreal::TCharPtr, TIsCharEncodingCompatibleWithTCHAR>::Value': Formatting string must be a TCHAR array.
    //     error: cannot pass object of non-trivial type 'Unreal::TCharPtr' through variadic function; call will abort at runtime [-Wnon-pod-varargs]
    UE_LOG(LogSpear, Log, TEXT("%s"), *Unreal::toFString(str));
}

std::string Log::getPrefix(const std::filesystem::path& current_file, int current_line)
{
    // TODO: remove platform-specific logic
    #if BOOST_COMP_MSVC
        return "[SPEAR | " + getCurrentFileAbbreviated(current_file) + ":" + std::format("{:04}", current_line) + "] ";
    #elif BOOST_COMP_CLANG
        return "[SPEAR | " + getCurrentFileAbbreviated(current_file) + ":" + (boost::format("%04d")%current_line).str() + "] ";
    #else
        #error
    #endif
}

std::string Log::getCurrentFileAbbreviated(const std::filesystem::path& current_file)
{
    return current_file.filename().string();
}

std::string Log::getCurrentFunctionAbbreviated(const std::string& current_function)
{
    // This function expects an input string in the format used by the BOOST_CURRENT_FUNCTION macro, which
    // can vary depending on the compiler.
    //
    // MSVC:
    //     __cdecl MyClass::MyClass(const class MyInputType1 &, const class MyInputType2 &, ...)
    //     MyReturnType __cdecl MyClass::myFunction<MyReturnType>(const class MyInputType1 &, const class MyInputType2 &, ...)
    //
    // Clang:
    //     MyClass::MyClass(const MyInputType1 &, const MyInputType2 &, ...)
    //     virtual MyReturnType MyClass::myFunction()
    //
    // Due to this variability, the most robust strategy for obtaining a sensible abbreviated function name
    // seems to be the following: replace all template expressions and function arguments with simplified
    // strings, then tokenize, then return the token that contains "(" and ")".
    
    // Make a copy of the input string so we can simplify it in-place.
    std::string current_function_simplified = current_function;

    // Iteratively simplify template expressions with "<...>". We do this iteratively, because regular
    // expressions are not intended to handle arbitrarily nested brackets.
    std::regex template_expression_regex("<(([a-zA-Z0-9_:*&,.{}() ])|(<\\.\\.\\.>))+>");

    // Keep iterating until the string doesn't change.
    std::string current_function_more_simplified;
    while (current_function_more_simplified != current_function_simplified) {
        current_function_more_simplified = current_function_simplified;
        current_function_simplified = std::regex_replace(current_function_simplified, template_expression_regex, "<...>");
    }

    // Simplify function arguments, either with "()" or "(...)".
    std::regex function_void_arguments_regex("\\(void\\)");
    current_function_simplified = std::regex_replace(current_function_simplified, function_void_arguments_regex, "()");

    std::regex function_non_void_arguments_regex("\\((([a-zA-Z0-9_:*&,.{} ])|(<\\.\\.\\.>))+\\)");
    current_function_simplified = std::regex_replace(current_function_simplified, function_non_void_arguments_regex, "(...)");

    // Either return the token ending in ::operator (indicating we're inside a lambda) and the following
    // token, or the token containing "(" and ")".
    std::vector<std::string> tokens = Std::tokenize(current_function_simplified, "*& ");
    for (int i = 0; i < tokens.size(); i++) {
        std::string& current_token = tokens.at(i);

        if (i < tokens.size() - 1) {
            std::string& next_token = tokens.at(i + 1);
            if (Std::endsWith(current_token, "::operator")) {
                return current_token + " " + next_token;
            }
        }

        if (Std::contains(current_token, "(") && Std::contains(current_token, ")")) {
            return current_token;
        }
    }

    // If our simplification strategy didn't work for some reason, then just return the input to facilitate
    // further debugging.
    return current_function;
}
