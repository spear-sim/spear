#include "CoreUtils.h"
#include "Config.h"

void CoreUtils::StartupModule()
{
    Config::initialize();
}

void CoreUtils::ShutdownModule()
{
    Config::terminate();
}

IMPLEMENT_MODULE(CoreUtils, CoreUtils)
