//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Modules/ModuleInterface.h>

#include "SpServices/EngineService.h"
#include "SpServices/LegacyService.h"
#include "SpServices/Rpclib.h"
#include "SpServices/SpFuncService.h"
#include "SpServices/UnrealService.h"

class UWorld;

class SpServices : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

private:
    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);
    void worldBeginPlayHandler();

    UWorld* world_ = nullptr;

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;
    FDelegateHandle world_begin_play_handle_;

    std::unique_ptr<rpc::server> rpc_server_ = nullptr;
    std::unique_ptr<EngineService<rpc::server>> engine_service_ = nullptr;
    std::unique_ptr<LegacyService> legacy_service_ = nullptr;
    std::unique_ptr<SpFuncService> sp_func_service_ = nullptr;
    std::unique_ptr<UnrealService> unreal_service_ = nullptr;
};
