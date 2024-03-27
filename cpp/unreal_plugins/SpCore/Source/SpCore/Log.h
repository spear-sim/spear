//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <filesystem>
#include <string>
#include <utility> // std::forward

#include <CoreGlobals.h> // IsRunningCommandlet

#include "SpCore/Std.h"
#include "SpCore/Boost.h"

// This macro returns an empty string if __VA_ARGS__ is empty, and returns __VA_ARGS__ with a leading comma otherwise. This macro
// enables variadic macro arguments to be passed into a function that accepts other arguments, regardless of whether or not
// __VA_ARGS__ is empty. It must be defined conditionally because MSVC doesn't support __VA_OPT__ by default.
#if BOOST_COMP_MSVC
    #define SP_VA_ARGS_WITH_LEADING_COMMA(...) , __VA_ARGS__
#elif BOOST_COMP_CLANG
    #define SP_VA_ARGS_WITH_LEADING_COMMA(...) __VA_OPT__(,) __VA_ARGS__
#else
    #error
#endif

// In some situations, the output from UE_LOG is not available, e.g., running on a cluster through an RL framework like RLLib.
// In other situations, the output from std::cout is not available, e.g., running in the editor or debugging in Visual Studio.
// It is therefore desirable to have a logging system that writes to both locations. We provide the following SP_LOG macros
// for this purpose. These need to be macros rather than functions, because they interact with __FILE__ and __LINE__ and
// BOOST_CURRENT_FUNCTION, similar to our assert implementation. In future, we could make the logging targets more configurable,
// but for now, we simply write to UE_LOG if we're in the editor (i.e., if WITH_EDITOR evaluates to true and IsRunningCommandlet()
// returns false) and std::cout otherwise.
#define SP_LOG(...)               Log::log(__FILE__, __LINE__ SP_VA_ARGS_WITH_LEADING_COMMA(__VA_ARGS__))
#define SP_LOG_CURRENT_FUNCTION() Log::logCurrentFunction(__FILE__, __LINE__, BOOST_CURRENT_FUNCTION)

// Helper macro that can be useful when printing to the game viewport or some other target.
#define SP_LOG_GET_PREFIX() Log::getPrefix(__FILE__, __LINE__)

class SPCORE_API Log
{
public:
    Log() = delete;
    ~Log() = delete;

    static void log(const std::filesystem::path& current_file, int current_line, auto&&... args)
    {
        std::string str = getPrefix(current_file, current_line) + toString(std::forward<decltype(args)>(args)...);
        
        #if WITH_EDITOR // defined in an auto-generated header
            if (IsRunningCommandlet()) {
                logStdout(str); // editor mode via command-line (e.g., during cooking)
            } else {
                logUnreal(str); // editor mode via GUI
            }
        #else
            logStdout(str); // standalone mode
        #endif
    }

    static void logCurrentFunction(const std::filesystem::path& current_file, int current_line, const std::string& current_function);
    static std::string getPrefix(const std::filesystem::path& current_file, int current_line);

private:
    static std::string toString()
    {
        return "";
    }

    static std::string toString(auto&&... args)
    {
        return Std::toString(std::forward<decltype(args)>(args)...);
    }

    static void logStdout(const std::string& str);
    static void logUnreal(const std::string& str);

    static std::string getCurrentFileAbbreviated(const std::filesystem::path& current_file);
    static std::string getCurrentFunctionAbbreviated(const std::string& current_function);
};
