#include "Config.h"

#include <Containers/StringConv.h>
#include <Misc/CommandLine.h>
#include <Misc/Parse.h>
#include <Misc/Paths.h>

YAML::Node Config::config_;

void Config::initialize()
{
    FString config_file;

    // If a config file is provided via the command-line, then load it
    if (FParse::Value(FCommandLine::Get(), TEXT("config_file="), config_file)) {
        config_ = YAML::LoadFile(TCHAR_TO_UTF8(*config_file));

    // Otherwise, if MyProject/Temp/config.yaml exists, then load it
    } else if (FPaths::FileExists(FPaths::ConvertRelativePathToFull(FPaths::ProjectDir().Append("Temp/config.yaml")))) {
        // Read config file contents into a YAML::Node
        config_file = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir().Append("Temp/config.yaml"));
        config_ = YAML::LoadFile(TCHAR_TO_UTF8(*config_file));
    }

    // Otherwise assert
    else {
        ASSERT(false);
    }
}

void Config::terminate()
{
    config_.reset();
}
