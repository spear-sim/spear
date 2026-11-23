//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <mutex>  // std::lock_guard, std::recursive_mutex
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
extern SPCORE_API std::recursive_mutex g_config_mutex;

class SPCORE_API Config
{
public:
    Config() = delete;
    ~Config() = delete;

    // If the -sp-config-file= command-line argument is passed in to the executable, calling requestInitialize()
    // will successfully initialize the config system. Otherwise, calling requestInitialize() will fail to
    // initialize the config system. If the config system isn't initialized, then get(...) returns a
    // default-constructed value, so systems that need a different fallback value must check if the config system
    // has been successfully initialized by calling Config::isInitialized(). Calling terminate() will completely
    // reset the state of the config system, regardless of whether or not it was successfully initialized.
    //
    // The config system can be accessed from any thread, and can be re-initialized while other threads are
    // accessing it, e.g., by ASpInitializeWorldManager on the game thread while the RPC worker thread is reading
    // config values. So we protect it with g_config_mutex. We use a recursive mutex because get(...) can log
    // while holding the mutex, and logging can read config values on the same thread (see OutputLog.cpp).

    static void requestInitialize();
    static void initialize(const std::string& config_file);
    static void terminate();
    static bool isInitialized();

    // The get(...) function is used to extract a value from the config system. This function takes as input
    // the fully qualified key that leads to the required config value. For example, if you have a config.yaml
    // such as:
    //
    //     SIMULATOR:
    //         TIME_DELTA_SECONDS: 0.1
    //
    // and you want to access the configuration parameter SIMULATOR.TIME_DELTA_SECONDS, you would
    // need to call this function as follows:
    //
    //     float time_delta_seconds = Config::get<float>("SIMULATOR.TIME_DELTA_SECONDS");

    template <typename TValue>
    static TValue get(const std::string& key)
    {
        std::lock_guard<std::recursive_mutex> lock(g_config_mutex);

        TValue value = TValue();
        if (isInitialized()) {
            try {
                value = Yaml::get<TValue>(g_config_node, key);
            } catch (...) {
                SP_LOG_CURRENT_FUNCTION();
                SP_LOG("    ERROR: Couldn't get value for key: ", key);
                SP_ASSERT(false);
            }
        }
        return value;
    }

    template <typename TValue>
    static TValue get(const std::vector<std::string>& keys)
    {
        std::lock_guard<std::recursive_mutex> lock(g_config_mutex);

        TValue value = TValue();
        if (isInitialized()) {
            try {
                value = Yaml::get<TValue>(g_config_node, keys);
            } catch (...) {
                SP_LOG_CURRENT_FUNCTION();
                SP_LOG("    ERROR: Couldn't get value for keys: [", Std::join(keys, ", "), "]");
                SP_ASSERT(false);
            }
        }
        return value;
    }

private:
    inline static bool s_initialized_ = false;
};
