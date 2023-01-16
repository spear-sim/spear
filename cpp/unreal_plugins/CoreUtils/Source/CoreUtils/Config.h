//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "CoreUtils/Assert.h"
#include "CoreUtils/YamlCpp.h"

class COREUTILS_API Config
{
public:
    static void initialize();
    static void terminate();

    //
    // This function is used to extract a value from the Config system. This function takes as input
    // the fully qualified key that leads to the required config value. For example, if you have a
    // config.yaml such as...
    //
    // SIMULATOR:
    //   TIME_DELTA_SECONDS: 0.1
    //
    // ...and you want to access to access the configuration parameter SIMULATOR.TIME_DELTA_SECONDS, you
    // would need to call this function as follows...
    //
    // float time_delta_seconds = Config::get<float>("SIMULATOR.TIME_DELTA_SECONDS");
    //

    template <typename TValue>
    static TValue get(const std::string& key)
    {
        ASSERT(key != "");

        std::stringstream ss(key);
        std::string s;
        std::vector<std::string> keys;

        while (std::getline(ss, s, '.')) {
            keys.push_back(s);
        }

        return getFromKeys<TValue>(keys);
    }

    template <typename TValue>
    static TValue getFromKeys(const std::vector<std::string>& keys)
    {
        // At least one key should be present when this function is called
        ASSERT(keys.size() > 0);

        // Make sure we have s_config_ defined before trying to read from it
        ASSERT(s_config_.IsDefined());

        YAML::Node node = s_config_;
        for (auto& key : keys) {
            // If key doesn't exist, then print an informative error message and assert
            if (!node[key]) {
                std::cout << "[SPEAR | Config.h] Invalid key, keys == [";
                for (int i = 0; i < keys.size() - 1; i++) {
                    std::cout << "\"" << keys.at(i) << "\", ";
                }
                std::cout << "\"" << keys.at(keys.size() - 1) << "\"], key " << key << " is invalid." << std::endl;
                ASSERT(false);
            }

            // We don't use node = node[key], because operator= merges the right-hand side into the left-hand
            // side Node. Also, repeated assignment to the same Node consumes additional memory as noted in
            // https://github.com/jbeder/yaml-cpp/issues/502.
            node.reset(node[key]);
        }

        return node.as<TValue>();
    }
    
private:
    Config() = default;
    ~Config() = default;

    static YAML::Node s_config_;
};
