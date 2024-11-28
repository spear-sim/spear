//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <GameMapsSettings.h>

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

class InitializeEngineService : public Service {
public:
    InitializeEngineService() = delete;
    InitializeEngineService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        // Override default game map settings. We want to do this early enough to avoid loading the default map
        // specified in Unreal's config system, but late enough that that the UGameMapSettings default object
        // has already been initialized.

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_GAME_DEFAULT_MAP")) {
            std::string game_default_map = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.GAME_DEFAULT_MAP");

            SP_LOG("Overriding game default map...");
            SP_LOG("Old game default map: ", Unreal::toStdString(UGameMapsSettings::GetGameDefaultMap()));
            SP_LOG("New game default map: ", game_default_map);

            UGameMapsSettings::SetGameDefaultMap(Unreal::toFString(game_default_map));
        }

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_GLOBAL_DEFAULT_GAME_MODE")) {
            std::string global_default_game_mode = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.GLOBAL_DEFAULT_GAME_MODE");

            SP_LOG("Overriding global default game mode...");
            SP_LOG("Old global default game mode: ", Unreal::toStdString(UGameMapsSettings::GetGlobalDefaultGameMode()));
            SP_LOG("New global default game mode: ", global_default_game_mode);

            UGameMapsSettings::SetGlobalDefaultGameMode(Unreal::toFString(global_default_game_mode));
        }
    }

    ~InitializeEngineService() override = default;
};
