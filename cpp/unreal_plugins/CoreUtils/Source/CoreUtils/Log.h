//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <filesystem>
#include <string>
#include <utility>

#include <boost/current_function.hpp>

#include "CoreUtils/Std.h"

// In some situations, the output from UE_LOG is not available, e.g., running on a cluster through an RL framework like RLLib.
// In other situations, the output from std::cout is not available, e.g., running in the editor or debugging in Visual Studio.
// It is therefore desirable to have a logging system that writes to both locations. We provide the following SP_LOG macros
// for this purpose. These need to be macros rather than functions, because they interact with __FILE__ and BOOST_CURRENT_FUNCTION,
// similar to our assert implementation. In future, we could make the logging targets configurable, but for now, we simply
// write to UE_LOG if we're in editor mode (i.e., if WITH_EDITOR evaluates to true) and std::cout otherwise.
#define SP_LOG(...)               Log::log(__FILE__, __VA_ARGS__)
#define SP_LOG_CURRENT_FUNCTION() Log::logCurrentFunction(__FILE__, BOOST_CURRENT_FUNCTION)

// Helper macro that can be useful when printing to the game viewport or some other target
#define SP_LOG_GET_PREFIX() Log::getPrefix(__FILE__)

class COREUTILS_API Log
{
public:
    template <class... TArgs>
    static void log(const std::filesystem::path& current_file, TArgs&&... args)
    {
        std::string str = getPrefix(current_file) + Std::toString(std::forward<TArgs>(args)...);
        #if WITH_EDITOR
            logUnreal(str);
        #else
            logStdout(str);
        #endif
    }

    static void logCurrentFunction(const std::filesystem::path& current_file, const std::string& current_function);

    static std::string getPrefix(const std::filesystem::path& current_file);

private:
    static void logStdout(const std::string& str);
    static void logUnreal(const std::string& str);

    static std::string getCurrentFileAbbreviated(const std::filesystem::path& current_file);
    static std::string getCurrentFunctionAbbreviated(const std::string& current_function);
};
