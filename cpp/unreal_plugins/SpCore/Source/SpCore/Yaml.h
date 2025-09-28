//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/YamlCpp.h"

class Yaml
{
public:
    Yaml() = delete;
    ~Yaml() = delete;

    // The get(...) function is used to extract a value from a YAML node. This function takes as input the
    // fully qualified key that leads to the required config value. For example, if you have a config.yaml
    // such as:
    //
    //     SIMULATOR:
    //         TIME_DELTA_SECONDS: 0.1
    //
    // and you want to access to access the configuration parameter SIMULATOR.TIME_DELTA_SECONDS, you would
    // need to call this function as follows:
    //
    //     float time_delta_seconds = Yaml::get<float>(node, "SIMULATOR.TIME_DELTA_SECONDS");

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

    static std::string toString(const YAML::Node& node)
    {
        YAML::Emitter emitter;
        emitter << node;
        return emitter.c_str();
    }
};
