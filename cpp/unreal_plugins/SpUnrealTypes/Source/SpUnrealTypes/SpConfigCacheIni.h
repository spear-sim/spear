//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>    // std::map
#include <string> // std::string

#include <Containers/Array.h>
#include <Containers/UnrealString.h> // FString
#include <CoreGlobals.h>             // GConfig, GEngineIni, GGameIni, GInputIni, GScalabilityIni, etc
#include <HAL/Platform.h>            // PLATFORM_DESKTOP
#include <Kismet/BlueprintFunctionLibrary.h>
#include <Misc/ConfigCacheIni.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpConfigCacheIni.generated.h"

//
// Each Filename argument is a short name (e.g., "Engine", "Game", "Scalability") rather than the value of the
// corresponding global variable (e.g., GEngineIni, GGameIni, GScalabilityIni). We store pointers to these global
// variables because they might not be initialized yet when this class is loaded.
//

UCLASS()
class USpConfigCacheIni : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="SPEAR")
    static bool GetSection(const FString& Section, TArray<FString>& Result, const FString& Filename)
    {
        SP_ASSERT(GConfig);

        std::string filename = Unreal::toStdString(Filename);
        SP_ASSERT(Std::containsKey(s_filenames_, filename));
        FString* global_filename = s_filenames_.at(filename);

        // INI files that aren't loaded in this build are mapped to nullptr, in which case we fail softly in the same
        // way as GConfig->GetSection(...) fails when a section doesn't exist.
        if (global_filename) {
            return GConfig->GetSection(*Section, Result, *global_filename);
        } else {
            Result.Reset();
            return false;
        }
    }

private:
    // See Engine/Source/Runtime/Core/Public/CoreGlobals.h for an exhaustive list of global INI filenames.
    inline static std::map<std::string, FString*> s_filenames_ = {
        {"DeviceProfiles",   &GDeviceProfilesIni},
        {"Engine",           &GEngineIni},
        {"Game",             &GGameIni},
        {"GameplayTags",     &GGameplayTagsIni},
        {"GameUserSettings", &GGameUserSettingsIni},
        {"Hardware",         &GHardwareIni},
        {"Input",            &GInputIni},
        {"InstallBundle",    &GInstallBundleIni},
        {"RuntimeOptions",   &GRuntimeOptionsIni},
        {"Scalability",      &GScalabilityIni},

        // These INI files are only loaded in desktop builds, see LoadRemainingConfigFiles(...) in
        // Engine/Source/Runtime/Core/Private/Misc/ConfigCacheIni.cpp.
        #if PLATFORM_DESKTOP
            {"Compat",    &GCompatIni},
            {"Lightmass", &GLightmassIni},
        #else
            {"Compat",    nullptr},
            {"Lightmass", nullptr},
        #endif

        // These INI files are only loaded in editor builds, see LoadRemainingConfigFiles(...) in
        // Engine/Source/Runtime/Core/Private/Misc/ConfigCacheIni.cpp.
        #if WITH_EDITOR // defined in an auto-generated header
            {"Editor",            &GEditorIni},
            {"EditorKeyBindings", &GEditorKeyBindingsIni},
            {"EditorLayout",      &GEditorLayoutIni},
            {"EditorPerProject",  &GEditorPerProjectIni},
            {"EditorSettings",    &GEditorSettingsIni}
        #else
            {"Editor",            nullptr},
            {"EditorKeyBindings", nullptr},
            {"EditorLayout",      nullptr},
            {"EditorPerProject",  nullptr},
            {"EditorSettings",    nullptr}
        #endif
    };
};
