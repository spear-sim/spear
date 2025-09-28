//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <exception> // std::current_exception, std::rethrow_exception
#include <string>
#include <vector>

#include <HAL/Platform.h> // SPCORE_API

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Yaml.h"
#include "SpCore/YamlCpp.h"

//
// We need the variables below to be globals because they are referenced in templated code. This requirement
// arises because templated code gets compiled at each call site. If a call site is in a different module
// (i.e., outside of SpCore), and it references a static variable, then the module will get its own local
// copy of the static variable. This behavior only happens on Clang, because MSVC has a different methodology
// for handling static variables in shared libraries. See the link below for details:
//     https://stackoverflow.com/questions/31495877/i-receive-different-results-on-unix-and-win-when-use-static-members-with-static
//

extern SPCORE_API YAML::Node g_config_node;

class SPCORE_API Config
{
public:
    Config() = delete;
    ~Config() = delete;

    // If the -config_file= command-line argument is passed in to the executable, calling requestInitialize()
    // will successfully initialize the config system. Otherwise, calling requestInitialize() will fail to
    // initialize the config system. Systems that want to use the config system must check if it has been
    // successfully initialized by checking Config::isInitialized(). Calling terminate()  will completely reset
    // the state of the config system, regardless of whether or not it was successfully initialized.

    static void initialize(const std::string& config_file);
    static void requestInitialize();
    static void terminate();
    static bool isInitialized();

    // The get(...) function is used to extract a value from the config system. This function takes as input
    // the fully qualified key that leads to the required config value. For example, if you have a config.yaml
    // such as:
    //
    //     SIMULATOR:
    //         TIME_DELTA_SECONDS: 0.1
    //
    // and you want to access to access the configuration parameter SIMULATOR.TIME_DELTA_SECONDS, you would
    // need to call this function as follows:
    //
    //     float time_delta_seconds = Config::get<float>("SIMULATOR.TIME_DELTA_SECONDS");

    template <typename TValue>
    static TValue get(const std::string& key)
    {
        SP_ASSERT(isInitialized());

        try {
            return Yaml::get<TValue>(g_config_node, key);
        } catch (...) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    ERROR: Couldn't get value for key: ", key);
            std::rethrow_exception(std::current_exception());
            return TValue();
        }
    }

    template <typename TValue>
    static TValue get(const std::vector<std::string>& keys)
    {
        SP_ASSERT(isInitialized());

        try {
            return Yaml::get<TValue>(g_config_node, keys);
        } catch (...) {
            SP_LOG_CURRENT_FUNCTION();
            SP_LOG("    ERROR: Couldn't get value for keys: [", Std::join(keys, ", "), "]");
            std::rethrow_exception(std::current_exception());
            return TValue();
        }
    }

private:
    inline static bool s_initialized_ = false;
};
