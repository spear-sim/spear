//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <filesystem>
#include <mutex>   // std::recursive_mutex
#include <string>
#include <utility> // std::forward

#include <HAL/Platform.h>      // SPCORE_API
#include <Logging/LogMacros.h> // DECLARE_LOG_CATEGORY_EXTERN

#include "SpCore/Boost.h"
#include "SpCore/Std.h"

DECLARE_LOG_CATEGORY_EXTERN(LogSpear, Log, All);

//
// In some situations, the output from UE_LOG is not available, e.g., running on a cluster through an RL framework like RLLib.
// In other situations, the output from std::cout is not available, e.g., running in the editor or debugging in Visual Studio.
// It is therefore desirable to have a logging system that writes to both locations. We provide the following SP_LOG macros
// for this purpose. These need to be macros rather than functions, because they interact with __FILE__ and __LINE__ and
// BOOST_CURRENT_FUNCTION, similar to our assert implementation. In future, we could make the logging targets more configurable,
// but for now, we simply write to UE_LOG if we're in the editor (i.e., if WITH_EDITOR evaluates to true and IsRunningCommandlet()
// returns false) and std::cout otherwise.
//

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

    template <typename... TArgs>
    static void log(const std::filesystem::path& current_file, int current_line, TArgs&&... args)
    {
        std::string str = getPrefix(current_file, current_line) + Std::toString(std::forward<TArgs>(args)...);
        logString(str);
    }

    template <typename... TArgs>
    static void logNoPrefix(TArgs&&... args)
    {
        std::string str = Std::toString(std::forward<TArgs>(args)...);
        logString(str);
    }

    static void logCurrentFunction(const std::filesystem::path& current_file, int current_line, const std::string& current_function);
    static std::string getPrefix(const std::filesystem::path& current_file, int current_line);

    // Shared across Log::logStringToStdout(...), StdOutputDevice::Serialize(...) (see SpCore/OutputLog.cpp),
    // and Assert.cpp's own terminal reporting, so a full multi-part message from any one of these can never
    // be torn by an interleaved write from another, regardless of which thread each runs on (Unreal's GLog
    // can invoke output devices from a dedicated background thread rather than the thread that logged the
    // message). This needs to be a recursive_mutex because Assert.cpp's reporting code and this class's own
    // logString(...) can both re-enter the lock on the same thread, e.g., when the vendored PPK_ASSERT
    // print() function locks around its own raw fprintf(...) calls and then also calls SP_LOG_NO_PREFIX(...),
    // which locks again via logStringToStdout(...).
    static std::recursive_mutex& getStdoutMutex();

private:
    static void logString(const std::string& str);
    static void logStringToStdout(const std::string& str);
    static void logStringToUnreal(const std::string& str);

    static std::string getCurrentFileAbbreviated(const std::filesystem::path& current_file);
    static std::string getCurrentFunctionAbbreviated(const std::string& current_function);
};
