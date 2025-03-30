//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Engine/LocalPlayer.h>

// EnhancedInput
#include <EnhancedInputSubsystems.h> // UEnhancedInputLocalPlayerSubsystem
#include <InputAction.h>
#include <InputModifiers.h>          // UInputModifier
#include <InputTriggers.h>           // UInputTrigger

// MovieRenderPipeline
#include <LevelSequence.h>
#include <MoviePipelinePrimaryConfig.h>
#include <MoviePipelineQueue.h> // UMoviePipelineExecutorJob
#include <MoviePipelineQueueEngineSubsystem.h>

#include "SpCore/UnrealClassRegistrar.h"

#include "SpServices/Service.h"

class RegisterClassesService : public Service {
public:
    RegisterClassesService()
    {
        //
        // We can't register these types in SpCore, because if we set SpCore to depend on plugins like
        // EnhancedInput, then we get cooking errors, presumably because SpCore gets loaded before
        // these plugins.
        //

        // EnhancedInput

        UnrealClassRegistrar::registerSubsystemClass<UEnhancedInputLocalPlayerSubsystem, ULocalPlayer>("UEnhancedInputLocalPlayerSubsystem");

        UnrealClassRegistrar::registerClass<UInputAction>("UInputAction");

        UnrealClassRegistrar::registerClass<UInputModifierDeadZone>("UInputModifierDeadZone");
        UnrealClassRegistrar::registerClass<UInputModifierFOVScaling>("UInputModifierFOVScaling");
        UnrealClassRegistrar::registerClass<UInputModifierNegate>("UInputModifierNegate");
        UnrealClassRegistrar::registerClass<UInputModifierResponseCurveExponential>("UInputModifierResponseCurveExponential");
        UnrealClassRegistrar::registerClass<UInputModifierResponseCurveUser>("UInputModifierResponseCurveUser");
        UnrealClassRegistrar::registerClass<UInputModifierScalar>("UInputModifierScalar");
        UnrealClassRegistrar::registerClass<UInputModifierScaleByDeltaTime>("UInputModifierScaleByDeltaTime");
        UnrealClassRegistrar::registerClass<UInputModifierSmooth>("UInputModifierSmooth");
        UnrealClassRegistrar::registerClass<UInputModifierSmoothDelta>("UInputModifierSmoothDelta");
        UnrealClassRegistrar::registerClass<UInputModifierSwizzleAxis>("UInputModifierSwizzleAxis");
        UnrealClassRegistrar::registerClass<UInputModifierToWorldSpace>("UInputModifierToWorldSpace");

        UnrealClassRegistrar::registerClass<UInputTriggerCombo>("UInputTriggerCombo");
        UnrealClassRegistrar::registerClass<UInputTriggerChordAction>("UInputTriggerChordAction");
        UnrealClassRegistrar::registerClass<UInputTriggerChordBlocker>("UInputTriggerChordBlocker");
        UnrealClassRegistrar::registerClass<UInputTriggerDown>("UInputTriggerDown");
        UnrealClassRegistrar::registerClass<UInputTriggerHold>("UInputTriggerHold");
        UnrealClassRegistrar::registerClass<UInputTriggerHoldAndRelease>("UInputTriggerHoldAndRelease");
        UnrealClassRegistrar::registerClass<UInputTriggerPressed>("UInputTriggerPressed");
        UnrealClassRegistrar::registerClass<UInputTriggerPulse>("UInputTriggerPulse");
        UnrealClassRegistrar::registerClass<UInputTriggerReleased>("UInputTriggerReleased");
        UnrealClassRegistrar::registerClass<UInputTriggerTap>("UInputTriggerTap");
        UnrealClassRegistrar::registerClass<UInputTriggerTimedBase>("UInputTriggerTimedBase");

        // MovieRenderPipeline

        UnrealClassRegistrar::registerEngineSubsystemClass<UMoviePipelineQueueEngineSubsystem>("UMoviePipelineQueueEngineSubsystem");

        UnrealClassRegistrar::registerClass<ULevelSequence>("ULevelSequence");
        UnrealClassRegistrar::registerClass<UMoviePipelineExecutorJob>("UMoviePipelineExecutorJob");
        UnrealClassRegistrar::registerClass<UMoviePipelinePrimaryConfig>("UMoviePipelinePrimaryConfig");
    }

    ~RegisterClassesService() override
    {
        // EnhancedInput

        UnrealClassRegistrar::unregisterSubsystemClass<UEnhancedInputLocalPlayerSubsystem, ULocalPlayer>("UEnhancedInputLocalPlayerSubsystem");

        UnrealClassRegistrar::unregisterClass<UInputAction>("UInputAction");

        UnrealClassRegistrar::unregisterClass<UInputModifierDeadZone>("UInputModifierDeadZone");
        UnrealClassRegistrar::unregisterClass<UInputModifierFOVScaling>("UInputModifierFOVScaling");
        UnrealClassRegistrar::unregisterClass<UInputModifierNegate>("UInputModifierNegate");
        UnrealClassRegistrar::unregisterClass<UInputModifierResponseCurveExponential>("UInputModifierResponseCurveExponential");
        UnrealClassRegistrar::unregisterClass<UInputModifierResponseCurveUser>("UInputModifierResponseCurveUser");
        UnrealClassRegistrar::unregisterClass<UInputModifierScalar>("UInputModifierScalar");
        UnrealClassRegistrar::unregisterClass<UInputModifierScaleByDeltaTime>("UInputModifierScaleByDeltaTime");
        UnrealClassRegistrar::unregisterClass<UInputModifierSmooth>("UInputModifierSmooth");
        UnrealClassRegistrar::unregisterClass<UInputModifierSmoothDelta>("UInputModifierSmoothDelta");
        UnrealClassRegistrar::unregisterClass<UInputModifierSwizzleAxis>("UInputModifierSwizzleAxis");
        UnrealClassRegistrar::unregisterClass<UInputModifierToWorldSpace>("UInputModifierToWorldSpace");

        UnrealClassRegistrar::unregisterClass<UInputTriggerCombo>("UInputTriggerCombo");
        UnrealClassRegistrar::unregisterClass<UInputTriggerChordAction>("UInputTriggerChordAction");
        UnrealClassRegistrar::unregisterClass<UInputTriggerChordBlocker>("UInputTriggerChordBlocker");
        UnrealClassRegistrar::unregisterClass<UInputTriggerDown>("UInputTriggerDown");
        UnrealClassRegistrar::unregisterClass<UInputTriggerHold>("UInputTriggerHold");
        UnrealClassRegistrar::unregisterClass<UInputTriggerHoldAndRelease>("UInputTriggerHoldAndRelease");
        UnrealClassRegistrar::unregisterClass<UInputTriggerPressed>("UInputTriggerPressed");
        UnrealClassRegistrar::unregisterClass<UInputTriggerPulse>("UInputTriggerPulse");
        UnrealClassRegistrar::unregisterClass<UInputTriggerReleased>("UInputTriggerReleased");
        UnrealClassRegistrar::unregisterClass<UInputTriggerTap>("UInputTriggerTap");
        UnrealClassRegistrar::unregisterClass<UInputTriggerTimedBase>("UInputTriggerTimedBase");

        // MovieRenderPipeline

        UnrealClassRegistrar::unregisterEngineSubsystemClass<UMoviePipelineQueueEngineSubsystem>("UMoviePipelineQueueEngineSubsystem");

        UnrealClassRegistrar::unregisterClass<ULevelSequence>("ULevelSequence");
        UnrealClassRegistrar::unregisterClass<UMoviePipelineExecutorJob>("UMoviePipelineExecutorJob");
        UnrealClassRegistrar::unregisterClass<UMoviePipelinePrimaryConfig>("UMoviePipelinePrimaryConfig");
    }
};
