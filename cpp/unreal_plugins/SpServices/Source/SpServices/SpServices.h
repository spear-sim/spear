//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Modules/ModuleInterface.h>

#include "SpServices/Rpclib.h"

#include "SpServices/EngineService.h"
#include "SpServices/EnhancedInputService.h"
#include "SpServices/InitializeEngineService.h"
#include "SpServices/InputService.h"
#include "SpServices/LegacyService.h"
#include "SpServices/SpFuncService.h"
#include "SpServices/UnrealService.h"

class SpServices : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

private:
    std::unique_ptr<rpc::server> rpc_server_ = nullptr;

    std::unique_ptr<EngineService<rpc::server>> engine_service_ = nullptr;

    std::unique_ptr<EnhancedInputService> enhanced_input_service_ = nullptr;
    std::unique_ptr<InitializeEngineService> initialize_engine_service_ = nullptr;
    std::unique_ptr<InputService> input_service_ = nullptr;
    std::unique_ptr<LegacyService> legacy_service_ = nullptr;
    std::unique_ptr<SpFuncService> sp_func_service_ = nullptr;
    std::unique_ptr<UnrealService> unreal_service_ = nullptr;
};
