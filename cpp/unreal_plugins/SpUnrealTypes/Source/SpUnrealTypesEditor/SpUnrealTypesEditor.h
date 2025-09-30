//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Modules/ModuleInterface.h>

class SpUnrealTypesEditor : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

private:
    void registerClasses() const;
    void unregisterClasses() const;
};
