//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/EngineBaseTypes.h>       // EViewModeIndex
#include <HAL/Platform.h>                 // int32
#include <Internationalization/Text.h>    // FText
#include <MoviePipelineDeferredPasses.h>
#include <MovieRenderPipelineDataTypes.h> // FMoviePipelinePassIdentifier
#include <ShowFlags.h>                    // FEngineShowFlags
#include <UObject/ObjectMacros.h>         // GENERATED_BODY, UCLASS

#include "SpMoviePipelineDeferredPass.generated.h"

UCLASS()
class USpMoviePipelineDeferredPass : public UMoviePipelineDeferredPassBase
{
    GENERATED_BODY()

public:
    USpMoviePipelineDeferredPass() : UMoviePipelineDeferredPassBase()
    {
        PassIdentifier = FMoviePipelinePassIdentifier("SpecularDisabled");
    }

#if WITH_EDITOR
    virtual FText GetDisplayText() const override
    {
        return FText::FromString("Deferred Rendering (Specular Disabled)");
    }
#endif

    virtual void GetViewShowFlags(FEngineShowFlags& OutShowFlag, EViewModeIndex& OutViewModeIndex) const override
    {
        OutShowFlag = FEngineShowFlags(EShowFlagInitMode::ESFIM_Game);
        OutShowFlag.Specular = false;

        OutViewModeIndex = EViewModeIndex::VMI_Lit;
    }

    virtual int32 GetOutputFileSortingOrder() const override { return 2; }
};
