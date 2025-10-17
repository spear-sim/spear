//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/EngineBaseTypes.h>       // EViewModeIndex
#include <HAL/Platform.h>                 // int32
#include <HAL/IConsoleManager.h>          // EConsoleVariableFlags, IConsoleVariable
#include <Internationalization/Text.h>    // FText
#include <MoviePipelineDeferredPasses.h>
#include <MovieRenderPipelineDataTypes.h> // FMoviePipelinePassIdentifier, FMoviePipelineRenderPassMetrics
#include <ShowFlags.h>                    // EShowFlagInitMode, FEngineShowFlags
#include <UObject/ObjectMacros.h>         // GENERATED_BODY, UCLASS

#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpMoviePipelineDeferredPass.generated.h"

UCLASS()
class USpMoviePipelineDeferredPass : public UMoviePipelineDeferredPassBase
{
    GENERATED_BODY()

public:
    USpMoviePipelineDeferredPass() : UMoviePipelineDeferredPassBase()
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
        return Unreal::toFText("Deferred Rendering (Specular=0, MaterialAO=1)");
    }
#endif

    void GetViewShowFlags(FEngineShowFlags& out_show_flag, EViewModeIndex& out_view_mode_index) const override
    {
        out_show_flag = FEngineShowFlags(EShowFlagInitMode::ESFIM_Game);
        out_show_flag.SetSpecular(false);
        out_view_mode_index = EViewModeIndex::VMI_Lit;
    }

    int32 GetOutputFileSortingOrder() const override
    {
        return 2;
    }

    void RenderSample_GameThreadImpl(const FMoviePipelineRenderPassMetrics& in_sample_state) override
    {
        IConsoleVariable* material_ao_cvar = IConsoleManager::Get().FindConsoleVariable(*Unreal::toFString("r.Lumen.ScreenProbeGather.MaterialAO"));
        SP_ASSERT(material_ao_cvar);
        int material_ao_cvar_value = material_ao_cvar->GetInt();
        material_ao_cvar->Set(0);
        UMoviePipelineDeferredPassBase::RenderSample_GameThreadImpl(in_sample_state);
        material_ao_cvar->Set(material_ao_cvar_value);
    }
};
