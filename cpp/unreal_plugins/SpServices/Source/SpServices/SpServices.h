//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Modules/ModuleInterface.h>

#include "SpServices/Rpclib.h"

// EngineService.
#include "SpServices/EngineService.h"

// Services that don't require a reference to EngineService.
#include "SpServices/InitializeEngineService.h"
#include "SpServices/RegisterClassesService.h"

// Services that require a reference to EngineService.
#include "SpServices/EnhancedInputService.h"
#include "SpServices/InputService.h"
#include "SpServices/LegacyService.h"
#include "SpServices/SharedMemoryService.h"
#include "SpServices/UnrealService.h"

// Services that require a reference to EngineService and SharedMemoryService.
#include "SpServices/NavigationService.h"
#include "SpServices/SpFuncService.h"

class SpServices : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

private:
    std::unique_ptr<rpc::server> rpc_server_ = nullptr;

    // EngineService.
    std::unique_ptr<EngineService<rpc::server>> engine_service_ = nullptr;

    // Services that don't require a reference to EngineService.
    std::unique_ptr<InitializeEngineService> initialize_engine_service_ = nullptr;
    std::unique_ptr<RegisterClassesService> register_classes_service_ = nullptr;

    // Services that require a reference to EngineService.
    std::unique_ptr<EnhancedInputService> enhanced_input_service_ = nullptr;
    std::unique_ptr<InputService> input_service_ = nullptr;
    std::unique_ptr<LegacyService> legacy_service_ = nullptr;
    std::unique_ptr<SharedMemoryService> shared_memory_service_ = nullptr;
    std::unique_ptr<UnrealService> unreal_service_ = nullptr;

    // Services that require a reference to EngineService and SharedMemoryService.
    std::unique_ptr<NavigationService> navigation_service_ = nullptr;
    std::unique_ptr<SpFuncService> sp_func_service_ = nullptr;
};
