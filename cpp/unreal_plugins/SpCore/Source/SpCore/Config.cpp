//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/Config.h"

#include <Containers/UnrealString.h> // FString
#include <HAL/Platform.h>            // TEXT
#include <Misc/CommandLine.h>
#include <Misc/Parse.h>

#include "SpCore/Unreal.h"
#include "SpCore/YamlCpp.h"


//
// We need the variables below to be globals because they are referenced in templated code. This requirement
// arises because templated code gets compiled at each call site. If a call site is in a different module
// (i.e., outside of SpCore), and it references a static variable, then the module will get its own local
// copy of the static variable. This behavior only happens on Clang, because MSVC has a different methodology
// for handling static variables in shared libraries. See the link below for details:
//     https://stackoverflow.com/questions/31495877/i-receive-different-results-on-unix-and-win-when-use-static-members-with-static
//

YAML::Node g_config_node;


void Config::requestInitialize()
{
    FString config_file;

    // if a config file is provided via the command-line, then load it
    if (FParse::Value(FCommandLine::Get(), *Unreal::toFString("config_file="), config_file)) {
        SP_LOG("Found config file via the -config_file command-line argument: ", Unreal::toStdString(config_file));
        g_config_node = YAML::LoadFile(Unreal::toStdString(config_file));
        s_initialized_ = true;
    } else {
        s_initialized_ = false;
    }
}

void Config::terminate()
{
    g_config_node.reset();
    s_initialized_ = false;
}

bool Config::isInitialized()
{
    return s_initialized_;
}
