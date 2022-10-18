#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "IgnoreCompilerWarnings.h"
ENABLE_IGNORE_COMPILER_WARNINGS
#include <yaml-cpp/yaml.h>
DISABLE_IGNORE_COMPILER_WARNINGS

#include "Assert/Assert.h"

class COREUTILS_API Config
{
public:
    // This function is used to initialize config_node_ when UnrealRL module is loaded.
    static void initialize();

    // This function is used to clear config_node_ when UnrealRL module is  unloaded.
    static void terminate();

    // This function is used to extract a value from a yaml file.
    // This function takes in a list of strings ('Keys') that lead to the required config value in the yaml file. The keys have to be passed in the descending hierarchical order.
    // For example, if you have a config.yaml such as below; abc: x: 1 y: 2 , to access 'y', you need to provide {"abc", "y"} as your Keys and not {"y", "abc"}.
    template <typename T>
    static T getValue(const std::vector<std::string>& keys)
    {
        // At least one key should be present when this function is called.
        ASSERT(keys.size() > 0, "getValue<> called without any keys. This is not supported.");

        // Make sure we have the config_node_ defined before trying to read from it.
        ASSERT(config_node_.IsDefined());

        // Create a local copy of Config.
        YAML::Node node = config_node_;

        for (const auto& key : keys) {
            if (!node[key]) {
                std::cout << "Invalid key, keys == [";
                for (size_t j = 0; j < keys.size() - 1; ++j) {
                    std::cout << "\"" << keys.at(j) << "\", ";
                }
                std::cout << "\"" << keys.at(keys.size() - 1) << "\"], key " << key << " is invalid." << std::endl;
                ASSERT(false);
            }

            // We don't use node = node[key], because operator= merges the right-hand side into the left-hand side Node.
            // Also, repeated assignment to the same Node consumes additional memory as noted in https://github.com/jbeder/yaml-cpp/issues/502.
            node.reset(node[key]);
        }

        return node.as<T>();
    }
private:
    Config() = default;
    ~Config() = default;

    // Load contents of the config file intot YAML::Node.
    static YAML::Node config_node_;
};
