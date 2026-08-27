//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServicesEditor/SpServicesEditor.h"

#include <memory> // std::make_unique

#include <CoreGlobals.h>           // IsRunningCommandlet
#include <Misc/CoreDelegates.h>    // FCoreDelegates
#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

#include "SpCore/Assert.h"
#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Log.h"

// EngineService
#include "SpServices/EngineService.h"

// Services that require a reference to EngineService
#include "SpServicesEditor/UnrealServiceEditor.h"

// needed to use SpServices::engine_service_
#include "SpServices/RpcServer.h"
#include "SpServices/EngineService.h"
#include "SpServices/SpServices.h"

void SpServicesEditor::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_ASSERT_MODULE_LOADED("SpCoreEditor");
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

    post_engine_init_handle_ = FCoreDelegates::OnPostEngineInit.AddRaw(this, &SpServicesEditor::postEngineInitHandler);
    engine_pre_exit_handle_  = FCoreDelegates::OnEnginePreExit.AddRaw(this, &SpServicesEditor::enginePreExitHandler);
}

void SpServicesEditor::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet()) {
            return;
        }
    #endif

    FCoreDelegates::OnEnginePreExit.Remove(engine_pre_exit_handle_);
    FCoreDelegates::OnPostEngineInit.Remove(post_engine_init_handle_);
    engine_pre_exit_handle_.Reset();
    post_engine_init_handle_.Reset();
}

void SpServicesEditor::postEngineInitHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    // This can't happen in StartupModule() because if we re-open a SPEAR project in the editor, then
    // StartupModule() will get called again before the first ShutdownModule() gets called.

    SpServices* sp_services = FModuleManager::Get().GetModulePtr<SpServices>("SpServices");
    SP_ASSERT(sp_services);

    // We can't guarantee whether SpServices's or SpServicesEditor's OnPostEngineInit handler will run first,
    // so we call requestInitialize() here to guarantee that engine_service_ is valid, regardless of order.
    sp_services->requestInitialize();
    SP_ASSERT(sp_services->isInitialized());

    // Create editor world services.
    editor_unreal_service_editor_ = UnrealServiceEditor::create(sp_services->engine_service_.get());
}

void SpServicesEditor::enginePreExitHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    // Note that sp_services is not guaranteed to be initialized here, see above.

    SP_ASSERT(editor_unreal_service_editor_);
    editor_unreal_service_editor_ = nullptr;
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpServicesEditor, SpServicesEditor);
