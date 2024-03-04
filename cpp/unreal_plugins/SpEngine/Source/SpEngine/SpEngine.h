//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <memory> // std::unique_ptr

#include <Modules/ModuleInterface.h>

#include "SpCore/Rpclib.h"
#include "SpEngine/EngineService.h"
#include "SpEngine/GameWorldService.h"

class SpEngine : public IModuleInterface
{
public:
    void StartupModule() override;
    void ShutdownModule() override;

private:
    std::unique_ptr<rpc::server> rpc_server_ = nullptr;
    std::unique_ptr<EngineService<rpc::server>> engine_service_ = nullptr;
    std::unique_ptr<GameWorldService> game_world_service_ = nullptr;
};
