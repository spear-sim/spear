#pragma once

#include <CoreMinimal.h>
#include <Modules/ModuleManager.h>

class CoreUtils : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;
};
