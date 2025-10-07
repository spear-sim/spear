//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>
#include <vector>

#include <AI/NavigationSystemBase.h>
#include <Engine/Engine.h>                       // GEngine
#include <GameMapsSettings.h>
#include <GenericPlatform/GenericPlatformFile.h> // IPakFile
#include <Misc/App.h>
#include <Misc/CoreDelegates.h>
#include <NavMesh/RecastNavMesh.h>
#include <PhysicsEngine/PhysicsSettings.h>
#include <UObject/UObjectGlobals.h>              // GetMutableDefault

#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpServices/Service.h"

class InitializeEngineService : public Service
{
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

            UGameMapsSettings::SetGameDefaultMap(Unreal::toFString(game_default_map));

            SP_LOG("    New game default map: ", Unreal::toStdString(UGameMapsSettings::GetGameDefaultMap()));
        }

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_GLOBAL_DEFAULT_GAME_MODE")) {
            std::string global_default_game_mode = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.GLOBAL_DEFAULT_GAME_MODE");

            SP_LOG("    Overriding global default game mode...");
            SP_LOG("    Old global default game mode: ", Unreal::toStdString(UGameMapsSettings::GetGlobalDefaultGameMode()));

            UGameMapsSettings::SetGlobalDefaultGameMode(Unreal::toFString(global_default_game_mode));

            SP_LOG("    New global default game mode: ", Unreal::toStdString(UGameMapsSettings::GetGlobalDefaultGameMode()));
        }

        #if WITH_EDITORONLY_DATA // defined in an auto-generated header
            if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_EDITOR_STARTUP_MAP")) {
                std::string editor_startup_map = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.EDITOR_STARTUP_MAP");

                UGameMapsSettings* game_maps_settings = GetMutableDefault<UGameMapsSettings>();
                SP_ASSERT(game_maps_settings);

                SP_LOG("    Overriding editor startup map...");
                SP_LOG("    Old editor startup map: ", Unreal::toStdString(game_maps_settings->EditorStartupMap.GetLongPackageName()));

                game_maps_settings->EditorStartupMap = Unreal::toFString(editor_startup_map);

                SP_LOG("    New editor startup map: ", Unreal::toStdString(game_maps_settings->EditorStartupMap.GetLongPackageName()));
            }
        #endif

        //
        // Override benchmarking and fixed delta time
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_BENCHMARKING")) {
            bool benchmarking = Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.BENCHMARKING");

            SP_LOG("    Overriding benchmarking...");
            SP_LOG("    Old benchmarking: ", FApp::IsBenchmarking());

            FApp::SetBenchmarking(benchmarking);

            SP_LOG("    New benchmarking: ", FApp::IsBenchmarking());
        }

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_FIXED_DELTA_TIME")) {
            float fixed_delta_time = Config::get<float>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.FIXED_DELTA_TIME");

            SP_LOG("    Overriding fixed delta time...");
            SP_LOG("    Old fixed delta time: ", FApp::GetFixedDeltaTime());

            FApp::SetFixedDeltaTime(fixed_delta_time);

            SP_LOG("    New fixed delta time: ", FApp::GetFixedDeltaTime());
        }

        //
        // Override physics settings
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_PHYSICS_SETTINGS")) {
            std::string physics_settings_str = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.PHYSICS_SETTINGS_STRING");

            UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
            SP_ASSERT(physics_settings);

            SP_LOG("    Overriding physics settings...");
            SP_LOG("    Old physics settings: ");
            SP_LOG_NO_PREFIX(Unreal::getObjectPropertiesAsString(physics_settings));

            Unreal::setObjectPropertiesFromString(physics_settings, physics_settings_str);

            SP_LOG("    New physics settings: ");
            SP_LOG_NO_PREFIX(Unreal::getObjectPropertiesAsString(physics_settings));

            // validate
            if (FApp::IsBenchmarking() && physics_settings->bSubstepping) {
                double max_fixed_delta_time = physics_settings->MaxSubstepDeltaTime*physics_settings->MaxSubsteps;

                SP_LOG("    Validating physics settings...");
                SP_LOG("    Current fixed delta time:         ", FApp::GetFixedDeltaTime());
                SP_LOG("    Maximum allowed fixed delta time: ", max_fixed_delta_time);

                SP_ASSERT(FApp::GetFixedDeltaTime() <= max_fixed_delta_time);
            }
        }
    }

    ~InitializeEngineService()
    {
        SP_LOG_CURRENT_FUNCTION();

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.MOUNT_PAK_FILES") && FCoreDelegates::OnUnmountPak.IsBound()) {
            std::vector<std::string> pak_files = Config::get<std::vector<std::string>>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.PAK_FILES");
            int32 pak_order = 0;

            SP_LOG("    Unmounting PAK files...");
            for (auto& pak_file : pak_files) {
                SP_LOG("    Unmounting PAK file: ", pak_file);
                bool success = FCoreDelegates::OnUnmountPak.Execute(Unreal::toFString(pak_file));
                SP_ASSERT(success);
            }
        }
    }

protected:
    void postEngineInit()
    {
        SP_LOG_CURRENT_FUNCTION();

        Service::postEngineInit();

        //
        // Override navigation settings. We do this in postEngineInit() because we need GEngine to be
        // initialized.
        //

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_NAVIGATION_SYSTEM_DEFAULT_OBJECT")) {
            std::string navigation_system_default_object_str = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.NAVIGATION_SYSTEM_DEFAULT_OBJECT_STRING");

            SP_ASSERT(GEngine);

            UNavigationSystemBase* navigation_system_default_object = nullptr;
            if (GEngine->NavigationSystemClass.Get()) {
                navigation_system_default_object = GetMutableDefault<UNavigationSystemBase>(GEngine->NavigationSystemClass.Get());
            } else {
                GetMutableDefault<UNavigationSystemBase>();
            }
            SP_ASSERT(navigation_system_default_object);

            SP_LOG("    Overriding navigation system default object...");
            SP_LOG("    Old navigation system default object: ");
            SP_LOG_NO_PREFIX(Unreal::getObjectPropertiesAsString(navigation_system_default_object));

            Unreal::setObjectPropertiesFromString(navigation_system_default_object, navigation_system_default_object_str);

            SP_LOG("    New navigation system default object: ");
            SP_LOG_NO_PREFIX(Unreal::getObjectPropertiesAsString(navigation_system_default_object));
        }

        if (Config::isInitialized() && Config::get<bool>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_RECAST_NAV_MESH_DEFAULT_OBJECT")) {
            std::string recast_nav_mesh_default_object_str = Config::get<std::string>("SP_SERVICES.INITIALIZE_ENGINE_SERVICE.RECAST_NAV_MESH_DEFAULT_OBJECT_STRING");

            ARecastNavMesh* recast_nav_mesh_default_object = GetMutableDefault<ARecastNavMesh>();

            SP_LOG("    Overriding ARecastNavMesh default object...");
            SP_LOG("    Old ARecastNavMesh default object: ");
            SP_LOG_NO_PREFIX(Unreal::getObjectPropertiesAsString(recast_nav_mesh_default_object));

            Unreal::setObjectPropertiesFromString(recast_nav_mesh_default_object, recast_nav_mesh_default_object_str);

            SP_LOG("    New ARecastNavMesh default object: ");
            SP_LOG_NO_PREFIX(Unreal::getObjectPropertiesAsString(recast_nav_mesh_default_object));
        }
    }
};
