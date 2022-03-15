// #pragma once

// #include <string>
// #include <vector>

// #include "IgnoreCompilerWarnings.h"
// ENABLE_IGNORE_COMPILER_WARNINGS
// #include <yaml-cpp/yaml.h>
// DISABLE_IGNORE_COMPILER_WARNINGS

// #include "Assert.h"

// namespace unrealrl
// {
// class SIMULATIONCONTROLLER_API Config
// {
//     Config() = default;
//     ~Config() = default;

//     // load contents of the config file intot YAML::Node
//     static YAML::Node ConfigNode;

// public:
//     /**
//      * This function is used to initialize ConfigNode when UnrealRL module is
//      * loaded
//      */
//     static void Initialize();

//     /**
//      * This function is used to clear ConfigNode when UnrealRL module is
//      * unloaded
//      */
//     static void Terminate();

//     /**
//      * This function is used to extract a value from a yaml file.
//      * This function takes in a list of strings ('Keys') that lead to the
//      * required config value in the yaml file. The keys have to be passed in the
//      * descending hierarchical order. For example, if you have a config.yaml
//      * such as below; abc: x: 1 y: 2 , to access 'y', you need to provide
//      * {"abc", "y"} as your Keys and not {"y", "abc"}
//      */
//     template <typename T>
//     static T GetValue(const std::vector<std::string>& Keys)
//     {
//         // atleast one key should be present when this function is called
//         ASSERT(Keys.size() > 0);

//         // make sure we have the confignode defined before trying to read
//         // from it
//         ASSERT(ConfigNode.IsDefined());

//         // create a local copy of Config
//         YAML::Node Node = ConfigNode;

//         for (size_t i = 0; i < Keys.size(); ++i)
//         {
//             ASSERT(Node[Keys.at(i)]);

//             // We don't use Node = Node[Keys.at(i)], because operator= merges
//             // the right-hand side into the left-hand side Node. Also, repeated
//             // assignment to the same Node consumes additional memory as noted
//             // in https://github.com/jbeder/yaml-cpp/issues/502.
//             Node.reset(Node[Keys.at(i)]);
//         }

//         return Node.as<T>();
//     }
// };

// } // namespace unrealrl
