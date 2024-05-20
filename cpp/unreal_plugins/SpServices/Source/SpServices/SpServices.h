//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Modules/ModuleInterface.h>

#include "SpCore/Rpclib.h"

#include "SpServices/CppFuncService.h"
#include "SpServices/EngineService.h"
#include "SpServices/LegacyService.h"
#include "SpServices/UnrealService.h"

class SpServices : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

private:
    std::unique_ptr<rpc::server> rpc_server_ = nullptr;

    std::unique_ptr<EngineService<rpc::server>> engine_service_ = nullptr;

    std::unique_ptr<CppFuncService> cpp_func_service_ = nullptr;
    std::unique_ptr<LegacyService> legacy_service_ = nullptr;
    std::unique_ptr<UnrealService> unreal_service_ = nullptr;
};
