//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/YamlCpp.h"

class COREUTILS_API Config
{
public:
    Config() = delete;
    ~Config() = delete;

    // If the -config_file= command-line argument is passed in to the executable, calling initialize() will
    // successfully initialize the config system. Otherwise, calling initialize() will fail to initialize
    // the config system. Systems that want to use the config system must check if it has been successfully
    // initialized by checking the public static bool Config::s_initialized_ variable. Calling terminate() 
    // will completely reset the state of the config system, regardless of whether or not it was successfully
    // initialized.
    static void initialize();
    static void terminate();

    // The get(...) function is used to extract a value from the config system. This function takes as input
    // the fully qualified key that leads to the required config value. For example, if you have a config.yaml
    // such as...
    //
    // SIMULATOR:
    //   TIME_DELTA_SECONDS: 0.1
    //
    // ...and you want to access to access the configuration parameter SIMULATOR.TIME_DELTA_SECONDS, you
    // would need to call this function as follows...
    //
    // float time_delta_seconds = Config::get<float>("SIMULATOR.TIME_DELTA_SECONDS");

    template <typename TValue>
    static TValue get(const std::string& key)
    {
        SP_ASSERT(key != "");
        return getFromKeys<TValue>(Std::tokenize(key, "."));
    }

    template <typename TValue>
    static TValue getFromKeys(const std::vector<std::string>& keys)
    {
        // at least one key should be present when this function is called
        SP_ASSERT(keys.size() > 0);

        // make sure we have s_config_ defined before trying to read from it
        SP_ASSERT(s_config_.IsDefined());

        YAML::Node node = s_config_;
        for (auto& key : keys) {
            // if key doesn't exist, then print an informative error message and assert
            if (!node[key]) {
                std::string str = "Invalid key, keys == [";
                for (int i = 0; i < keys.size() - 1; i++) {
                    str = str + "\"" + keys.at(i)  + "\", ";
                }
                str = str + "\"" + keys.at(keys.size() - 1) + "\"], key \"" + key + "\" is invalid.";
                SP_LOG(str);
                SP_ASSERT(false);
            }

            // We don't use node = node[key], because operator= merges the right-hand side into the left-hand
            // side Node. Also, repeated assignment to the same Node consumes additional memory as noted in
            // https://github.com/jbeder/yaml-cpp/issues/502.
            node.reset(node[key]);
        }

        return node.as<TValue>();
    }

    inline static bool s_initialized_ = false;

private:
    inline static YAML::Node s_config_;
};
