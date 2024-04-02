//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
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

    // These public get(...) functions are stateful, in the sense that they depend on s_config_node_.
    template <typename TValue>
    static TValue get(const std::string& key)
    {
        SP_ASSERT(isInitialized());
        return get<TValue>(s_config_node_, key);
    }

    template <typename TValue>
    static TValue get(const std::vector<std::string>& keys)
    {
        SP_ASSERT(isInitialized());
        return get<TValue>(s_config_node_, keys);
    }

private:
    // These private get(...) functions are stateless, in the sense that they require a YAML node to be passed
    // in. If we need YAML functionality in other parts of the code, we could move these methods into their own
    // class.
    template <typename TValue>
    static TValue get(const YAML::Node& node, const std::string& key)
    {
        SP_ASSERT(key != "");
        return get<TValue>(node, Std::tokenize(key, "."));
    }

    template <typename TValue>
    static TValue get(const YAML::Node& node, const std::vector<std::string>& keys)
    {
        SP_ASSERT(!keys.empty());
        SP_ASSERT(node.IsDefined());

        YAML::Node current_node = node;
        for (auto& key : keys) {
            SP_ASSERT(key != "");

            // If the key is invalid, print an informative error message.
            if (!current_node[key]) {
                std::string str = "Invalid key, keys == [";
                for (int i = 0; i < keys.size() - 1; i++) {
                    str = str + "\"" + keys.at(i)  + "\", ";
                }
                str = str + "\"" + keys.at(keys.size() - 1) + "\"], key \"" + key + "\" is invalid.";
                SP_LOG(str);
                SP_ASSERT(false);
            }

            // We don't use current_node = current_node[key], because operator= merges the right-hand side
            // into the left-hand side Node. Also, repeated assignment to the same Node consumes additional
            // memory, as noted here:
            //     https://github.com/jbeder/yaml-cpp/issues/502.
            current_node.reset(current_node[key]);
        }

        return current_node.as<TValue>();
    }

    inline static YAML::Node s_config_node_;
    inline static bool s_initialized_ = false;
};
