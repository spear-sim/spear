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

class GameMapSettingsService : public Service {
public:
    GameMapSettingsService() = delete;
    GameMapSettingsService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        // Override default game map settings. We want to do this early enough to avoid loading the default map
        // specified in Unreal's config system, but late enough that that the UGameMapSettings default object
        // has already been initialized.
        std::string game_default_map;
        std::string global_default_game_mode;
        if (Config::isInitialized()) {
            game_default_map = Config::get<std::string>("SP_SERVICES.GAME_MAP_SETTINGS_SERVICE.GAME_DEFAULT_MAP");
            global_default_game_mode = Config::get<std::string>("SP_SERVICES.GAME_MAP_SETTINGS_SERVICE.GLOBAL_DEFAULT_GAME_MODE");
        }

        if (game_default_map != "") {
            SP_LOG("Overriding game default map: ", game_default_map);
            UGameMapsSettings::SetGameDefaultMap(Unreal::toFString(game_default_map));
        }

        if (global_default_game_mode != "") {
            SP_LOG("Overriding global default game mode: ", global_default_game_mode);
            UGameMapsSettings::SetGlobalDefaultGameMode(Unreal::toFString(global_default_game_mode));
        }
    }

    ~GameMapSettingsService() override = default;
};
