// #include "Config.h"

// #include "IgnoreCompilerWarnings.h"
// ENABLE_IGNORE_COMPILER_WARNINGS
// #include <yaml-cpp/yaml.h>
// DISABLE_IGNORE_COMPILER_WARNINGS

// #include <Misc/CommandLine.h>
// #include <Misc/Parse.h>
// #include <Misc/Paths.h>

// #include "Assert.h"

// YAML::Node unrealrl::Config::ConfigNode;

// void unrealrl::Config::Initialize()
// {
//     // path to the file that contains config values
//     FString ConfigFile;

//     // load ConfigNode from config file provided via command line
//     if (FParse::Value(FCommandLine::Get(), TEXT("configfile="), ConfigFile))
//     {
//         // read config file contents into a YAML::Node
//         ConfigNode = YAML::LoadFile(TCHAR_TO_UTF8(*ConfigFile));
//     }
//     // load ConfigNode from project dir
//     else if (FPaths::FileExists(FPaths::ConvertRelativePathToFull(
//                  FPaths::ProjectDir().Append("config.yaml"))))
//     {
//         // read config file contents into a YAML::Node
//         ConfigFile = FPaths::ConvertRelativePathToFull(
//             FPaths::ProjectDir().Append("config.yaml"));
//         ConfigNode = YAML::LoadFile(TCHAR_TO_UTF8(*ConfigFile));
//     }
//     else
//     {
//         // Currently we do not support launching the project without ConfigFile
//         // param.
//         ASSERT(false);
//     }
// }

// void unrealrl::Config::Terminate()
// {
//     ConfigNode.reset();
// }
