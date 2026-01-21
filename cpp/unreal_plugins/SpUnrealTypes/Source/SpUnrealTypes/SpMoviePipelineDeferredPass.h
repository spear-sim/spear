//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <utility> // std::make_pair, std::pair
#include <vector>

#include <Components/SceneCaptureComponent.h> // FEngineShowFlagsSetting
#include <Containers/Array.h>
#include <Containers/EnumAsByte.h>
#include <Containers/UnrealString.h>          // FString
#include <Engine/Engine.h>                    // GEngine
#include <Engine/EngineBaseTypes.h>           // EViewModeIndex
#include <HAL/Platform.h>                     // int32
#include <HAL/IConsoleManager.h>              // EConsoleVariableFlags, IConsoleVariable
#include <Internationalization/Text.h>        // FText
#include <Misc/DefaultValueHelper.h>
#include <MoviePipelineDeferredPasses.h>
#include <MovieRenderPipelineDataTypes.h>     // FMoviePipelineConsoleVariableEntry, FMoviePipelinePassIdentifier, FMoviePipelineRenderPassMetrics
#include <Sections/MovieSceneCVarSection.h>
#include <ShowFlags.h>                        // EShowFlagInitMode, FEngineShowFlags
#include <UObject/ObjectMacros.h>             // GENERATED_BODY, UCLASS, UPROPERTY
#include <UObject/ScriptInterface.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include <SpCore/Std.h>
#include "SpCore/Unreal.h"

#include "SpMoviePipelineDeferredPass.generated.h"

UCLASS()
class USpMoviePipelineDeferredPass : public UMoviePipelineDeferredPassBase
{
    GENERATED_BODY()
public:
    USpMoviePipelineDeferredPass()
    {
        SP_LOG_CURRENT_FUNCTION();
        PassIdentifier = FMoviePipelinePassIdentifier("SPEAR");
    }

    ~USpMoviePipelineDeferredPass() override
    {
        SP_LOG_CURRENT_FUNCTION();
    }

#if WITH_EDITOR // defined in an auto-generated header
    FText GetDisplayText() const override
    {
        return Unreal::toFText("Deferred Rendering (SPEAR customizations)");
    }
#endif

    void GetViewShowFlags(FEngineShowFlags& out_show_flag, EViewModeIndex& out_view_mode_index) const override
    {
        UMoviePipelineDeferredPassBase::GetViewShowFlags(out_show_flag, out_view_mode_index);

        for (auto& engine_show_flag_setting : EngineShowFlagSettings) {
            int32 index = out_show_flag.FindIndexByName(Unreal::toTCharPtr(engine_show_flag_setting.ShowFlagName));
            SP_ASSERT(index != -1);
            out_show_flag.SetSingleFlag(index, engine_show_flag_setting.Enabled);
        }

        if (bOverrideViewModeIndex) {
            out_view_mode_index = ViewModeIndex;
        }
    }

    int32 GetOutputFileSortingOrder() const override
    {
        return 2;
    }

    void RenderSample_GameThreadImpl(const FMoviePipelineRenderPassMetrics& in_sample_state) override
    {
        std::vector<std::pair<FString, float>> previous_cvar_values;

        // save the value of each cvar
        for (auto& cvar_entry : ConsoleVariables) {
            if (cvar_entry.bIsEnabled) {
                IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr(cvar_entry.Name)); 
                SP_ASSERT(cvar);
                float previous_cvar_value = cvar->GetFloat();
                previous_cvar_values.push_back(std::make_pair(cvar_entry.Name, previous_cvar_value));
                if (bVerbose) {
                    SP_LOG(Unreal::toStdString(PassIdentifier.Name), ": setting console variable ", Unreal::toStdString(cvar_entry.Name), " to value ", cvar_entry.Value, " (previous value ", previous_cvar_value, ")");
                }
                cvar->Set(cvar_entry.Value, EConsoleVariableFlags::ECVF_SetByConsole); // EConsoleVariableFlags::ECVF_SetByConsole is highest-priority
            }
        }

        for (auto& cmd : PreRenderConsoleCommands) {
            SP_ASSERT(GEngine);
            if (bVerbose) {
                SP_LOG("Execucting console command: ", Unreal::toStdString(cmd));
            }
            GEngine->Exec(GetWorld(), Unreal::toTCharPtr(cmd));
        }

        UMoviePipelineDeferredPassBase::RenderSample_GameThreadImpl(in_sample_state);

        for (auto& cmd : PostRenderConsoleCommands) {
            SP_ASSERT(GEngine);
            if (bVerbose) {
                SP_LOG("Execucting console command: ", Unreal::toStdString(cmd));
            }
            GEngine->Exec(GetWorld(), Unreal::toTCharPtr(cmd));
        }

        // restore previous cvar values in reverse order
        for (int i = previous_cvar_values.size() - 1; i >= 0; i--) {
            const auto& [cvar_name, cvar_value] = Std::at(previous_cvar_values, i);
            IConsoleVariable* cvar = IConsoleManager::Get().FindConsoleVariable(Unreal::toTCharPtr(cvar_name)); 
            SP_ASSERT(cvar);
            if (bVerbose) {
                SP_LOG(Unreal::toStdString(PassIdentifier.Name), ": restoring console variable ", Unreal::toStdString(cvar_name), " to value ", cvar_value);
            }
            cvar->Set(cvar_value, EConsoleVariableFlags::ECVF_SetByConsole); // EConsoleVariableFlags::ECVF_SetByConsole is highest-priority
        }
    }

    // Allow customizing FEngineShowFlags
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TArray<FEngineShowFlagsSetting> EngineShowFlagSettings;

    // Allow customizing EViewModeIndex
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bOverrideViewModeIndex = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TEnumAsByte<EViewModeIndex> ViewModeIndex = EViewModeIndex::VMI_Lit;

    // Allow customizing console variables
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TArray<FMoviePipelineConsoleVariableEntry> ConsoleVariables;

    // Allow console commands
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TArray<FString> PreRenderConsoleCommands;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    TArray<FString> PostRenderConsoleCommands;

    // Enable debug printing
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SPEAR")
    bool bVerbose = false;
};

// Define some extra passes because each distinct type will only show up once in the MPPC editor

UCLASS()
class USpMoviePipelineDeferredPass_ExtraPass0 : public USpMoviePipelineDeferredPass
{
    GENERATED_BODY()
public:
    USpMoviePipelineDeferredPass_ExtraPass0() { SP_LOG_CURRENT_FUNCTION(); PassIdentifier = FMoviePipelinePassIdentifier("SPEAR_ExtraPass0"); }
    ~USpMoviePipelineDeferredPass_ExtraPass0() override { SP_LOG_CURRENT_FUNCTION(); }
#if WITH_EDITOR // defined in an auto-generated header
    FText GetDisplayText() const override { return Unreal::toFText("Deferred Rendering (SPEAR customizations extra pass 0)"); }
#endif
};

UCLASS()
class USpMoviePipelineDeferredPass_ExtraPass1 : public USpMoviePipelineDeferredPass
{
    GENERATED_BODY()
public:
    USpMoviePipelineDeferredPass_ExtraPass1() { SP_LOG_CURRENT_FUNCTION(); PassIdentifier = FMoviePipelinePassIdentifier("SPEAR_ExtraPass1"); }
    ~USpMoviePipelineDeferredPass_ExtraPass1() override { SP_LOG_CURRENT_FUNCTION(); }
#if WITH_EDITOR // defined in an auto-generated header
    FText GetDisplayText() const override { return Unreal::toFText("Deferred Rendering (SPEAR customizations extra pass 1)"); }
#endif
};

UCLASS()
class USpMoviePipelineDeferredPass_ExtraPass2 : public USpMoviePipelineDeferredPass
{
    GENERATED_BODY()
public:
    USpMoviePipelineDeferredPass_ExtraPass2() { SP_LOG_CURRENT_FUNCTION(); PassIdentifier = FMoviePipelinePassIdentifier("SPEAR_ExtraPass2"); }
    ~USpMoviePipelineDeferredPass_ExtraPass2() override { SP_LOG_CURRENT_FUNCTION(); }
#if WITH_EDITOR // defined in an auto-generated header
    FText GetDisplayText() const override { return Unreal::toFText("Deferred Rendering (SPEAR customizations extra pass 2)"); }
#endif
};

UCLASS()
class USpMoviePipelineDeferredPass_ExtraPass3 : public USpMoviePipelineDeferredPass
{
    GENERATED_BODY()
public:
    USpMoviePipelineDeferredPass_ExtraPass3() { SP_LOG_CURRENT_FUNCTION(); PassIdentifier = FMoviePipelinePassIdentifier("SPEAR_ExtraPass3"); }
    ~USpMoviePipelineDeferredPass_ExtraPass3() override { SP_LOG_CURRENT_FUNCTION(); }
#if WITH_EDITOR // defined in an auto-generated header
    FText GetDisplayText() const override { return Unreal::toFText("Deferred Rendering (SPEAR customizations extra pass 3)"); }
#endif
};
