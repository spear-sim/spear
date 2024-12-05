//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <filesystem>
#include <string>
#include <utility> // std::forward

#include <HAL/Platform.h> // SPCORE_API

#include "SpCore/Boost.h"
#include "SpCore/Std.h"

// In some situations, the output from UE_LOG is not available, e.g., running on a cluster through an RL framework like RLLib.
// In other situations, the output from std::cout is not available, e.g., running in the editor or debugging in Visual Studio.
// It is therefore desirable to have a logging system that writes to both locations. We provide the following SP_LOG macros
// for this purpose. These need to be macros rather than functions, because they interact with __FILE__ and __LINE__ and
// BOOST_CURRENT_FUNCTION, similar to our assert implementation. In future, we could make the logging targets more configurable,
// but for now, we simply write to UE_LOG if we're in the editor (i.e., if WITH_EDITOR evaluates to true and IsRunningCommandlet()
// returns false) and std::cout otherwise.
#define SP_LOG(...)               Log::log(__FILE__, __LINE__ __VA_OPT__(,) __VA_ARGS__)
#define SP_LOG_CURRENT_FUNCTION() Log::logCurrentFunction(__FILE__, __LINE__, BOOST_CURRENT_FUNCTION)
#define SP_LOG_NO_PREFIX(...)     Log::logNoPrefix(__VA_ARGS__)

// Helper macro that can be useful when printing to the game viewport or some other target.
#define SP_LOG_GET_PREFIX() Log::getPrefix(__FILE__, __LINE__)

class SPCORE_API Log
{
public:
    Log() = delete;
    ~Log() = delete;

    static void log(const std::filesystem::path& current_file, int current_line, auto&&... args)
    {
        std::string str = getPrefix(current_file, current_line) + Std::toString(std::forward<decltype(args)>(args)...);        
        logString(str);
    }

    static void logNoPrefix(auto&&... args)
    {
        std::string str = Std::toString(std::forward<decltype(args)>(args)...);
        logString(str);
    }

    static void logCurrentFunction(const std::filesystem::path& current_file, int current_line, const std::string& current_function);
    static std::string getPrefix(const std::filesystem::path& current_file, int current_line);

private:
    static void logString(const std::string& str);
    static void logStringToStdout(const std::string& str);
    static void logStringToUnreal(const std::string& str);

    static std::string getCurrentFileAbbreviated(const std::filesystem::path& current_file);
    static std::string getCurrentFunctionAbbreviated(const std::string& current_function);
};
