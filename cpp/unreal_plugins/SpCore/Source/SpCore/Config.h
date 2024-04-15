//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Yaml.h"
#include "SpCore/YamlCpp.h"

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
        return Yaml::get<TValue>(s_config_node_, key);
    }

    template <typename TValue>
    static TValue get(const std::vector<std::string>& keys)
    {
        SP_ASSERT(isInitialized());
        return Yaml::get<TValue>(s_config_node_, keys);
    }

private:
    inline static YAML::Node s_config_node_;
    inline static bool s_initialized_ = false;
};
