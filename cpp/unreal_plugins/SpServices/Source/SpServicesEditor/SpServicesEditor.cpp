//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServicesEditor/SpServicesEditor.h"

#include <memory> // std::make_unique

#include <CoreGlobals.h>           // IsRunningCommandlet
#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/Assert.h"
#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"

#include "SpServices/SpServices.h"

// Services that require a reference to EngineService
#include "SpServicesEditor/UnrealServiceEditor.h"

void SpServicesEditor::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_ASSERT_MODULE_LOADED("SpServices");
    SP_LOG_CURRENT_FUNCTION();

    // If we're cooking, then return early. In this case, there is no need to launch our services, and if
    // if we attempt to launch the RPC server while cooking, and the editor or game is already open, then we
    // will get an error because the port is in use.
    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet()) {
            return;
        }
    #endif

    SpServices* sp_services = FModuleManager::Get().GetModulePtr<SpServices>("SpServices");
    SP_ASSERT(sp_services);

    // Create editor world services.
    editor_unreal_service_editor_ = std::make_unique<UnrealServiceEditor>(sp_services->engine_service_.get(), sp_services->editor_world_filter_.get());
}

void SpServicesEditor::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet()) {
            return;
        }
    #endif

    SP_ASSERT(editor_unreal_service_editor_);
    editor_unreal_service_editor_ = nullptr;
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpServicesEditor, SpServicesEditor);
