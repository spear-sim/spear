//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include <GameMapsSettings.h>
#include <GenericPlatform/GenericPlatformFile.h> // IPakFile
#include <Misc/CoreDelegates.h>

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpServices/Service.h"

class InitializeEngineService : public Service {
public:
    InitializeEngineService() : Service("InitializeEngineService")
    {
        SP_LOG_CURRENT_FUNCTION();

        // Mount PAK files. PAK files in default locations are mounted before SpCore::StartupModule(), so
        // there is no danger of attempting to mount PAK files too early. See the function below for guidance
        // on how to set pak_order in Engine/Source/Runtime/PakFile/Private/IPlatformFilePak.cpp.
        //
        //     int32 FPakPlatformFile::GetPakOrderFromPakFilePath(const FString& PakFilePath)
        //     {
        //         if (PakFilePath.StartsWith(FString::Printf(TEXT("%sPaks/%s-"), *FPaths::ProjectContentDir(), FApp::GetProjectName())))
        //             return 4;
        //         else if (PakFilePath.StartsWith(FPaths::ProjectContentDir()))
        //             return 3;
        //         else if (PakFilePath.StartsWith(FPaths::EngineContentDir()))
        //             return 2;
        //         else if (PakFilePath.StartsWith(FPaths::ProjectSavedDir()))
        //             return 1;
        //         return 0;
        //     }

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.MOUNT_PAK_FILES") && FCoreDelegates::MountPak.IsBound()) {
            std::vector<std::string> pak_files = Config::get<std::vector<std::string>>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.PAK_FILES");
            int32 pak_order = 0;

            SP_LOG("    Mounting PAK files...");
            for (auto& pak_file : pak_files) {
                SP_LOG("    Mounting PAK file: ", pak_file);
                IPakFile* pak = FCoreDelegates::MountPak.Execute(Unreal::toFString(pak_file), pak_order);
                SP_ASSERT(pak);
            }
        }

        // Override default game map settings. We want to do this early enough to avoid loading the default map
        // specified in Unreal's config system, but late enough that that the UGameMapSettings default object
        // has already been initialized.

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_GAME_DEFAULT_MAP")) {
            std::string game_default_map = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.GAME_DEFAULT_MAP");

            SP_LOG("    Overriding game default map...");
            SP_LOG("    Old game default map: ", Unreal::toStdString(UGameMapsSettings::GetGameDefaultMap()));
            SP_LOG("    New game default map: ", game_default_map);

            UGameMapsSettings::SetGameDefaultMap(Unreal::toFString(game_default_map));
        }

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_GLOBAL_DEFAULT_GAME_MODE")) {
            std::string global_default_game_mode = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.GLOBAL_DEFAULT_GAME_MODE");

            SP_LOG("    Overriding global default game mode...");
            SP_LOG("    Old global default game mode: ", Unreal::toStdString(UGameMapsSettings::GetGlobalDefaultGameMode()));
            SP_LOG("    New global default game mode: ", global_default_game_mode);

            UGameMapsSettings::SetGlobalDefaultGameMode(Unreal::toFString(global_default_game_mode));
        }
    }
};
