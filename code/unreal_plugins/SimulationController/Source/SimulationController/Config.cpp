#include "Config.h"

#include "IgnoreCompilerWarnings.h"
ENABLE_IGNORE_COMPILER_WARNINGS
#include <yaml-cpp/yaml.h>
DISABLE_IGNORE_COMPILER_WARNINGS

#include <Misc/CommandLine.h>
#include <Misc/Parse.h>
#include <Misc/Paths.h>

#include "Assert.h"

YAML::Node Config::config_node_;

void Config::initialize()
{
    // Path to the file that contains config values.
    FString config_file;

    // Load config_node_ from config file provided via command line.
    if (FParse::Value(FCommandLine::Get(), TEXT("configfile="), config_file))
    {
        // Read config file contents into a YAML::Node.
        config_node_ = YAML::LoadFile(TCHAR_TO_UTF8(*config_file));
    }
    // Load config_node_ from project dir.
    else if (FPaths::FileExists(FPaths::ConvertRelativePathToFull(FPaths::ProjectDir().Append("Temp/config.yaml"))))
    {
        // Read config file contents into a YAML::Node.
        config_file = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir().Append("Temp/config.yaml"));
        config_node_ = YAML::LoadFile(TCHAR_TO_UTF8(*config_file));
    }
    else
    {
        ASSERT(false, "Did not receive a config file location from python and could not find config.yaml in project's 'Temp' directory. So, tried to launch project without a config file. Currently we do not support this.");
    }
}

void Config::terminate()
{
    config_node_.reset();
}
