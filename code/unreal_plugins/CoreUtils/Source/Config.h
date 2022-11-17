#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "CompilerWarningUtils.h"
BEGIN_IGNORE_COMPILER_WARNINGS
#include <yaml-cpp/yaml.h>
END_IGNORE_COMPILER_WARNINGS

#include "Assert/Assert.h"

class COREUTILS_API Config
{
public:
    static void initialize();
    static void terminate();

    //
    // This function is used to extract a value from the Config system. This function takes in a list of
    // keys that lead to the required config value. The keys need to be passed in the descending order.
    // For example, if you have a config.yaml such as...
    //
    // SIMULATOR:
    //   TIME_DELTA_SECONDS: 0.1
    //
    // ...and you want to access to access the configuration parameter SIMULATOR.TIME_DELTA_SECONDS, you
    // would need to call this function as follows...
    //
    // float time_delta_seconds = Config::getValue<float>({"SIMULATOR", "TIME_DELTA_SECONDS"});
    //

    template <typename T>
    static T getValue(const std::vector<std::string>& keys)
    {
        // At least one key should be present when this function is called
        ASSERT(keys.size() > 0);

        // Make sure we have the config_node_ defined before trying to read from it
        ASSERT(config_.IsDefined());

        YAML::Node node = config_;
        for (auto& key : keys) {

            // If key doesn't exist, then print an informative error message and assert
            if (!node[key]) {
                std::cout << "Invalid key, keys == [";
                for (int i = 0; i < keys.size() - 1; i++) {
                    std::cout << "\"" << keys.at(i) << "\", ";
                }
                std::cout << "\"" << keys.at(keys.size() - 1) << "\"], key " << key << " is invalid." << std::endl;
                ASSERT(false);
            }

            //
            // We don't use node = node[key], because operator= merges the right-hand side into the left-hand
            // side Node. Also, repeated assignment to the same Node consumes additional memory as noted in
            // https://github.com/jbeder/yaml-cpp/issues/502.
            //

            node.reset(node[key]);
        }

        return node.as<T>();
    }
private:
    Config() = default;
    ~Config() = default;

    static YAML::Node config_;
};
