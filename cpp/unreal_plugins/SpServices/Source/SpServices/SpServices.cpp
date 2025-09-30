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
#include "SpCore/UnrealClassRegistry.h"

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

void SpServices::registerClasses() const
{
    // ChaosVehiclesPlugin classes
    UnrealClassRegistry::registerActorClass<AWheeledVehiclePawn>("AWheeledVehiclePawn");
    UnrealClassRegistry::registerComponentClass<UChaosVehicleMovementComponent>("UChaosVehicleMovementComponent");

    // EnhancedInput classes
    UnrealClassRegistry::registerSubsystemClass<UEnhancedInputLocalPlayerSubsystem, ULocalPlayer>("UEnhancedInputLocalPlayerSubsystem");
    UnrealClassRegistry::registerClass<UInputAction>("UInputAction");
    UnrealClassRegistry::registerClass<UInputModifierDeadZone>("UInputModifierDeadZone");
    UnrealClassRegistry::registerClass<UInputModifierFOVScaling>("UInputModifierFOVScaling");
    UnrealClassRegistry::registerClass<UInputModifierNegate>("UInputModifierNegate");
    UnrealClassRegistry::registerClass<UInputModifierResponseCurveExponential>("UInputModifierResponseCurveExponential");
    UnrealClassRegistry::registerClass<UInputModifierResponseCurveUser>("UInputModifierResponseCurveUser");
    UnrealClassRegistry::registerClass<UInputModifierScalar>("UInputModifierScalar");
    UnrealClassRegistry::registerClass<UInputModifierScaleByDeltaTime>("UInputModifierScaleByDeltaTime");
    UnrealClassRegistry::registerClass<UInputModifierSmooth>("UInputModifierSmooth");
    UnrealClassRegistry::registerClass<UInputModifierSmoothDelta>("UInputModifierSmoothDelta");
    UnrealClassRegistry::registerClass<UInputModifierSwizzleAxis>("UInputModifierSwizzleAxis");
    UnrealClassRegistry::registerClass<UInputModifierToWorldSpace>("UInputModifierToWorldSpace");
    UnrealClassRegistry::registerClass<UInputTriggerCombo>("UInputTriggerCombo");
    UnrealClassRegistry::registerClass<UInputTriggerChordAction>("UInputTriggerChordAction");
    UnrealClassRegistry::registerClass<UInputTriggerChordBlocker>("UInputTriggerChordBlocker");
    UnrealClassRegistry::registerClass<UInputTriggerDown>("UInputTriggerDown");
    UnrealClassRegistry::registerClass<UInputTriggerHold>("UInputTriggerHold");
    UnrealClassRegistry::registerClass<UInputTriggerHoldAndRelease>("UInputTriggerHoldAndRelease");
    UnrealClassRegistry::registerClass<UInputTriggerPressed>("UInputTriggerPressed");
    UnrealClassRegistry::registerClass<UInputTriggerPulse>("UInputTriggerPulse");
    UnrealClassRegistry::registerClass<UInputTriggerReleased>("UInputTriggerReleased");
    UnrealClassRegistry::registerClass<UInputTriggerTap>("UInputTriggerTap");
    UnrealClassRegistry::registerClass<UInputTriggerTimedBase>("UInputTriggerTimedBase");

    // MovieRenderPipeline classes
    UnrealClassRegistry::registerEngineSubsystemClass<UMoviePipelineQueueEngineSubsystem>("UMoviePipelineQueueEngineSubsystem");
    UnrealClassRegistry::registerClass<UMoviePipelineExecutorJob>("UMoviePipelineExecutorJob");
    UnrealClassRegistry::registerClass<UMoviePipelinePrimaryConfig>("UMoviePipelinePrimaryConfig");
}

void SpServices::unregisterClasses() const
{
    // ChaosVehiclesPlugin classes
    UnrealClassRegistry::unregisterActorClass<AWheeledVehiclePawn>("AWheeledVehiclePawn");
    UnrealClassRegistry::unregisterComponentClass<UChaosVehicleMovementComponent>("UChaosVehicleMovementComponent");

    // EnhancedInput classes
    UnrealClassRegistry::unregisterSubsystemClass<UEnhancedInputLocalPlayerSubsystem, ULocalPlayer>("UEnhancedInputLocalPlayerSubsystem");
    UnrealClassRegistry::unregisterClass<UInputAction>("UInputAction");
    UnrealClassRegistry::unregisterClass<UInputModifierDeadZone>("UInputModifierDeadZone");
    UnrealClassRegistry::unregisterClass<UInputModifierFOVScaling>("UInputModifierFOVScaling");
    UnrealClassRegistry::unregisterClass<UInputModifierNegate>("UInputModifierNegate");
    UnrealClassRegistry::unregisterClass<UInputModifierResponseCurveExponential>("UInputModifierResponseCurveExponential");
    UnrealClassRegistry::unregisterClass<UInputModifierResponseCurveUser>("UInputModifierResponseCurveUser");
    UnrealClassRegistry::unregisterClass<UInputModifierScalar>("UInputModifierScalar");
    UnrealClassRegistry::unregisterClass<UInputModifierScaleByDeltaTime>("UInputModifierScaleByDeltaTime");
    UnrealClassRegistry::unregisterClass<UInputModifierSmooth>("UInputModifierSmooth");
    UnrealClassRegistry::unregisterClass<UInputModifierSmoothDelta>("UInputModifierSmoothDelta");
    UnrealClassRegistry::unregisterClass<UInputModifierSwizzleAxis>("UInputModifierSwizzleAxis");
    UnrealClassRegistry::unregisterClass<UInputModifierToWorldSpace>("UInputModifierToWorldSpace");
    UnrealClassRegistry::unregisterClass<UInputTriggerCombo>("UInputTriggerCombo");
    UnrealClassRegistry::unregisterClass<UInputTriggerChordAction>("UInputTriggerChordAction");
    UnrealClassRegistry::unregisterClass<UInputTriggerChordBlocker>("UInputTriggerChordBlocker");
    UnrealClassRegistry::unregisterClass<UInputTriggerDown>("UInputTriggerDown");
    UnrealClassRegistry::unregisterClass<UInputTriggerHold>("UInputTriggerHold");
    UnrealClassRegistry::unregisterClass<UInputTriggerHoldAndRelease>("UInputTriggerHoldAndRelease");
    UnrealClassRegistry::unregisterClass<UInputTriggerPressed>("UInputTriggerPressed");
    UnrealClassRegistry::unregisterClass<UInputTriggerPulse>("UInputTriggerPulse");
    UnrealClassRegistry::unregisterClass<UInputTriggerReleased>("UInputTriggerReleased");
    UnrealClassRegistry::unregisterClass<UInputTriggerTap>("UInputTriggerTap");
    UnrealClassRegistry::unregisterClass<UInputTriggerTimedBase>("UInputTriggerTimedBase");

    // MovieRenderPipeline classes
    UnrealClassRegistry::unregisterEngineSubsystemClass<UMoviePipelineQueueEngineSubsystem>("UMoviePipelineQueueEngineSubsystem");
    UnrealClassRegistry::unregisterClass<UMoviePipelineExecutorJob>("UMoviePipelineExecutorJob");
    UnrealClassRegistry::unregisterClass<UMoviePipelinePrimaryConfig>("UMoviePipelinePrimaryConfig");
}

// use IMPLEMENT_GAME_MODULE if module implements Unreal classes, use IMPLEMENT_MODULE otherwise
IMPLEMENT_GAME_MODULE(SpServices, SpServices);
