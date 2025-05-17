//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Modules/ModuleInterface.h>

class SpearSim : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;
};
