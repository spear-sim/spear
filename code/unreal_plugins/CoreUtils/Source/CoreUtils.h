#pragma once

#include <CoreMinimal.h>
#include <Modules/ModuleManager.h>

class CoreUtils : public IModuleInterface
{
public:
    // This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module.
    void StartupModule() override;

    // This function may be called during shutdown to clean up your module. For modules that support dynamic reloading, we call this function before unloading the module.
    void ShutdownModule() override;
};
