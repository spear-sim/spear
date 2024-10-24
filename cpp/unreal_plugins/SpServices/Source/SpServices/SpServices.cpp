//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/SpServices.h"

#include <memory> // std::make_unique, std::unique_ptr

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <Engine/World.h>                // FWorldDelegates
#include <GameMapsSettings.h>
#include <Modules/ModuleManager.h>       // IMPLEMENT_MODULE

#include "SpCore/Assert.h"
#include "SpCore/AssertModuleLoaded.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpServices/EngineService.h"
#include "SpServices/LegacyService.h"
#include "SpServices/Rpclib.h"
#include "SpServices/SpFuncService.h"
#include "SpServices/UnrealService.h"

void SpServices::StartupModule()
{
    SP_ASSERT_MODULE_LOADED("SpCore");
    SP_ASSERT_MODULE_LOADED("SpComponents");
    SP_ASSERT_MODULE_LOADED("UrdfRobot");
    SP_ASSERT_MODULE_LOADED("Vehicle");
    SP_LOG_CURRENT_FUNCTION();

    // Create RPC server object but don't launch it yet. We want to defer launching the server as late as
    // possible in worldBeginPlayHandler(), to give other plugins and game code a chance to register custom
    // types with UnrealClassRegistrar.
    int rpc_server_port = 30000;
    if (Config::isInitialized()) {
        rpc_server_port = Config::get<int>("SP_SERVICES.RPC_SERVER_PORT");
    }
    rpc_server_ = std::make_unique<rpc::server>(rpc_server_port);
    SP_ASSERT(rpc_server_);

    // EngineService needs its own custom logic for binding its entry points, because they are intended to
    // run directly on the RPC server worker thread, whereas all other entry points are intended to run on
    // work queues maintained by EngineService. So we pass in the RPC server when constructing EngineService,
    // and we pass in EngineService when constructing all other services.
    engine_service_ = std::make_unique<EngineService<rpc::server>>(rpc_server_.get());

    // Construct all other services by passing in EngineService.
    legacy_service_ = std::make_unique<LegacyService>(engine_service_.get());
    sp_func_service_ = std::make_unique<SpFuncService>(engine_service_.get());
    unreal_service_ = std::make_unique<UnrealService>(engine_service_.get());

    // Add callbacks after creating all other services. If other services add callbacks, we want to add ours
    // last, because this will guarantee that it gets called first. If our postWorldInitializationHandler(...)
    // callback gets called first, that means our worldBeginPlayHandler(...) callback will get called last,
    // which is the behavior we want, because we want to launch the RPC server as late as possible.
    post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SpServices::postWorldInitializationHandler);
    world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &SpServices::worldCleanupHandler);

    // Override the default game map. We want to do this early enough to avoid loading the default map
    // specified in Unreal's config system, but late enough that that the UGameMapSettings default object
    // has been initialized.
    std::string game_default_map;
    if (Config::isInitialized()) {
        game_default_map = Config::get<std::string>("SP_SERVICES.GAME_DEFAULT_MAP");
    }
    if (game_default_map != "") {
        SP_LOG("Overriding default game map: ", game_default_map);
        UGameMapsSettings::SetGameDefaultMap(Unreal::toFString(game_default_map));
    }
}

void SpServices::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
    FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

    world_cleanup_handle_.Reset();
    post_world_initialization_handle_.Reset();

    SP_ASSERT(unreal_service_);
    SP_ASSERT(sp_func_service_);
    SP_ASSERT(legacy_service_);
    unreal_service_ = nullptr;
    sp_func_service_ = nullptr;
    legacy_service_ = nullptr;

    SP_ASSERT(engine_service_);
    engine_service_->close();
    engine_service_ = nullptr;

    // not exactly symmetic with StartupModule because we deferred initialization until worldBeginPlayHandler()
    SP_ASSERT(rpc_server_);
    rpc_server_->close_sessions();
    rpc_server_->stop();
    rpc_server_ = nullptr;
}

void SpServices::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    SP_LOG("World name: ", Unreal::toStdString(world->GetName()));

    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
        SP_LOG("Caching world...");
        SP_ASSERT(!world_);
        world_ = world;
        world_begin_play_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &SpServices::worldBeginPlayHandler);
    }
}

void SpServices::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    SP_LOG("World name: ", Unreal::toStdString(world->GetName()));

    if (world == world_) {
        SP_LOG("Clearing cached world...");
        world_->OnWorldBeginPlay.Remove(world_begin_play_handle_);
        world_begin_play_handle_.Reset();
        world_ = nullptr;
    }
}

void SpServices::worldBeginPlayHandler()
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world_);

    // We defer launching the RPC server until here, to give other plugins and other game code as long as
    // possibe to register custom types with UnrealClassRegistar before the server is launched. This approach
    // guarantees that, if custom type is registered any time before this worldBeginPlayHandler() function,
    // e.g., in the StartupModule() function of a plugin or game module, then the custom type will be
    // accessible from Python for the entire lifetime of the RPC server.
    static bool once = false;
    if (!once) {
        once = true;
        int num_worker_threads = 1;
        rpc_server_->async_run(num_worker_threads);
    }
}

// use if module does not implement any Unreal classes
// IMPLEMENT_MODULE(SpComponents, SpComponents);

// use if module implements any Unreal classes
IMPLEMENT_GAME_MODULE(SpServices, SpServices);
