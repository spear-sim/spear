//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/SpServices.h"

#include <exception>
#include <memory> // std::make_unique

#include <CoreGlobals.h>           // IsRunningCommandlet
#include <Modules/ModuleManager.h> // FDefaultGameModuleImpl, FDefaultModuleImpl, IMPLEMENT_GAME_MODULE, IMPLEMENT_MODULE

// Unreal classes
#include <Engine/LocalPlayer.h>

// ChaosVehiclesPlugin classes
#include <WheeledVehiclePawn.h>
#include <ChaosVehicleMovementComponent.h>

// EnhancedInput classes
#include <EnhancedInputSubsystems.h> // UEnhancedInputLocalPlayerSubsystem
#include <InputAction.h>
#include <InputModifiers.h>          // UInputModifier
#include <InputTriggers.h>           // UInputTrigger

// MovieRenderPipeline classes
#include <MoviePipelinePrimaryConfig.h>
#include <MoviePipelineQueue.h> // UMoviePipelineExecutorJob
#include <MoviePipelineQueueEngineSubsystem.h>

#include "SpCore/Assert.h"
#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/UnrealClassRegistrar.h"

#include "SpServices/Service.h"

// RpcService
#include "SpServices/RpcService.h"

// EngineService
#include "SpServices/EngineService.h"

// Services that don't require a reference to EngineService
#include "SpServices/InitializeEngineService.h"

// Services that require a reference to EngineService
#include "SpServices/EnhancedInputService.h"
#include "SpServices/InitializeEditorWorldService.h"
#include "SpServices/InitializeGameWorldService.h"
#include "SpServices/InputService.h"
#include "SpServices/SharedMemoryService.h"
#include "SpServices/UnrealService.h"

// Services that require a reference to EngineService and SharedMemoryService
#include "SpServices/NavigationService.h"
#include "SpServices/SpFuncService.h"

void SpServices::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_LOG_CURRENT_FUNCTION();

    // If we're cooking, then return early. In this case, there is no need to launch our services, and if
    // if we attempt to launch the RPC server while cooking, and the editor or game is already open, then we
    // will get an error because the port is in use.
    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet()) {
            return;
        }
    #endif

    registerClasses();

    // Create world filters.
    editor_world_filter = std::make_unique<Service::EditorWorldFilter>();
    game_world_filter = std::make_unique<Service::GameWorldFilter>();

    // Create the RPC server. The RPC server won't be launched until beginFrame() to give other services a
    // chance to bind their entry points.
    rpc_service = std::make_unique<RpcService>();

    // EngineService needs its own custom logic for binding its entry points, because they are intended to
    // run directly on the RPC server worker thread, whereas most other entry points are intended to run on
    // work queues maintained by EngineService. So we pass in the RPC server when constructing EngineService,
    // and we pass in EngineService when constructing all other services that need to bind entry points. We
    // need to call engine_service->startup() explicitly.
    engine_service = std::make_unique<EngineService<rpc::server>>(rpc_service->rpc_server.get());
    engine_service->startup();

    // Construct services that don't require a reference to EngineService.
    initialize_engine_service = std::make_unique<InitializeEngineService>();

    // Construct services that require a reference to EngineService.
    enhanced_input_service = std::make_unique<EnhancedInputService>(engine_service.get());
    input_service = std::make_unique<InputService>(engine_service.get());
    shared_memory_service = std::make_unique<SharedMemoryService>(engine_service.get());

    // Construct services that require a reference to EngineService and SharedMemoryService.
    sp_func_service = std::make_unique<SpFuncService>(engine_service.get(), shared_memory_service.get());

    // Create editor world services.
    initialize_editor_world_service = std::make_unique<InitializeEditorWorldService>(engine_service.get(), editor_world_filter.get());
    editor_unreal_service = std::make_unique<UnrealService>(engine_service.get(), editor_world_filter.get());
    editor_navigation_service = std::make_unique<NavigationService>(engine_service.get(), shared_memory_service.get(), editor_world_filter.get());

    // Create game world services.
    initialize_game_world_service = std::make_unique<InitializeGameWorldService>(engine_service.get(), game_world_filter.get());
    game_unreal_service = std::make_unique<UnrealService>(engine_service.get(), game_world_filter.get());
    game_navigation_service = std::make_unique<NavigationService>(engine_service.get(), shared_memory_service.get(), game_world_filter.get());
}

void SpServices::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    #if WITH_EDITOR // defined in an auto-generated header
        if (IsRunningCommandlet()) {
            return;
        }
    #endif

    // The logic in EngineService guarantees that no other RPC server functions can ever get called after the
    // engine_service.terminate entry point gets called, which should have already happened at this point. So
    // it is ok to terminate our other services before terminating RpcService. This guarantee is necessary to
    // enable other plugins to implement their own services.

    SP_ASSERT(initialize_game_world_service);
    SP_ASSERT(game_unreal_service);
    SP_ASSERT(game_navigation_service);
    initialize_game_world_service = nullptr;
    game_unreal_service = nullptr;
    game_navigation_service = nullptr;

    SP_ASSERT(initialize_editor_world_service);
    initialize_editor_world_service = nullptr;
    editor_unreal_service = nullptr;
    editor_navigation_service = nullptr;

    SP_ASSERT(sp_func_service);
    sp_func_service = nullptr;

    SP_ASSERT(enhanced_input_service);
    SP_ASSERT(input_service);
    SP_ASSERT(shared_memory_service);
    enhanced_input_service = nullptr;
    input_service = nullptr;
    shared_memory_service = nullptr;

    SP_ASSERT(initialize_engine_service);
    initialize_engine_service = nullptr;

    // We need to call engine_service->shutdown() before shutting down the RPC server, so engine_service has
    // a chance to stop waiting on any futures that it might be waiting on. It will not be possible to shut
    // down the RPC server gracefully if a live entry point is waiting on a future.
    SP_ASSERT(engine_service);
    engine_service->shutdown();
    engine_service = nullptr;

    SP_ASSERT(rpc_service);
    rpc_service = nullptr;

    SP_ASSERT(editor_world_filter);
    SP_ASSERT(game_world_filter);
    editor_world_filter = nullptr;
    game_world_filter = nullptr;

    unregisterClasses();
}

// Normally we would do the operations in registerClasses() and unregisterClasses(...) in the opposite order.
// But we make an exception here (i.e., we do the operations in the same order) to make it easier and less
// error-prone to register classes.

void SpServices::registerClasses()
{
    // ChaosVehiclesPlugin classes
    UnrealClassRegistrar::registerActorClass<AWheeledVehiclePawn>("AWheeledVehiclePawn");
    UnrealClassRegistrar::registerComponentClass<UChaosVehicleMovementComponent>("UChaosVehicleMovementComponent");

    // EnhancedInput classes
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

    // MovieRenderPipeline classes
    UnrealClassRegistrar::registerEngineSubsystemClass<UMoviePipelineQueueEngineSubsystem>("UMoviePipelineQueueEngineSubsystem");
    UnrealClassRegistrar::registerClass<UMoviePipelineExecutorJob>("UMoviePipelineExecutorJob");
    UnrealClassRegistrar::registerClass<UMoviePipelinePrimaryConfig>("UMoviePipelinePrimaryConfig");
}

void SpServices::unregisterClasses()
{
    // ChaosVehiclesPlugin classes
    UnrealClassRegistrar::unregisterActorClass<AWheeledVehiclePawn>("AWheeledVehiclePawn");
    UnrealClassRegistrar::unregisterComponentClass<UChaosVehicleMovementComponent>("UChaosVehicleMovementComponent");

    // EnhancedInput classes
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

    // MovieRenderPipeline classes
    UnrealClassRegistrar::unregisterEngineSubsystemClass<UMoviePipelineQueueEngineSubsystem>("UMoviePipelineQueueEngineSubsystem");
    UnrealClassRegistrar::unregisterClass<UMoviePipelineExecutorJob>("UMoviePipelineExecutorJob");
    UnrealClassRegistrar::unregisterClass<UMoviePipelinePrimaryConfig>("UMoviePipelinePrimaryConfig");
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpServices, SpServices);
