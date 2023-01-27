//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "CoreUtils/Config.h"

#include <Misc/CommandLine.h>
#include <Misc/Parse.h>
#include <Misc/Paths.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Unreal.h"
#include "CoreUtils/YamlCpp.h"

YAML::Node Config::s_config_;

void Config::initialize()
{
    FString config_file;

    // If a config file is provided via the command-line, then load it
    if (FParse::Value(FCommandLine::Get(), TEXT("config_file="), config_file)) {
        s_config_ = YAML::LoadFile(Unreal::toStdString(config_file));

    // Otherwise, if MyProject/Temp/config.yaml exists, then load it
    } else if (FPaths::FileExists(FPaths::ConvertRelativePathToFull(FPaths::ProjectDir().Append("Temp/config.yaml")))) {
        config_file = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir().Append("Temp/config.yaml"));
        s_config_ = YAML::LoadFile(Unreal::toStdString(config_file));

    // Otherwise assert
    } else {
        ASSERT(false);
    }
}

void Config::terminate()
{
    s_config_.reset();
}
