//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Modules/ModuleInterface.h>

// Services that require a reference to EngineService
#include "SpServicesEditor/UnrealServiceEditor.h"

class SpServicesEditor : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

    // Editor world services
    std::unique_ptr<UnrealServiceEditor> editor_unreal_service_editor = nullptr;
};
