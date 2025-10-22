//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpProxyComponentManager.h"

#include <stddef.h> // uint32_t

#include <map>
#include <set>
#include <string>
#include <vector>

#include <boost/predef.h> // BOOST_COMP_CLANG, BOOST_COMP_MSVC

#include <GameFramework/Actor.h>
#include <Engine/Engine.h>      // GEngine
#include <Engine/EngineTypes.h> // EEndPlayReason
#include <Engine/World.h>

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

// TODO: remove platform-specific include
#if BOOST_COMP_MSVC
    #include <format>
#endif

class UActorComponent;
class UMaterialInterface;

ASpProxyComponentManager::ASpProxyComponentManager()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = true;
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

ASpProxyComponentManager::~ASpProxyComponentManager()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpProxyComponentManager::BeginPlay()
{
    SP_LOG_CURRENT_FUNCTION();

    AActor::BeginPlay();

    if (bIsInitialized) {
        request_reinitialize_ = true;
    }

    bIsInitialized = false;
    RegisteredProxyComponentDescIds.Empty();
    RegisteredProxyComponentDescNames.Empty();
}

void ASpProxyComponentManager::EndPlay(const EEndPlayReason::Type end_play_reason)
{
    SP_LOG_CURRENT_FUNCTION();

    Terminate();
    AActor::EndPlay(end_play_reason);
}

void ASpProxyComponentManager::Tick(float delta_time)
{
    AActor::Tick(delta_time);

    std::vector<AActor*> actors = Unreal::findActors(GetWorld());

    //
    // We must take care to handle the case where an ASpProxyComponentManager actor in the editor world has
    // already been initialized, and we press play in the editor. In this case, the proxy components that
    // were created in the editor world will be duplicated in the PIE world, but they will not have been
    // correctly registered with the ASpProxyComponentManager in the PIE world. So we destroy these
    // components and re-initialize. We can't do this re-initialization step in BeginPlay() because there
    // might be some actors and components that haven't been created yet.
    //
    // We also need to do a full tear-down and re-initialize step here, as opposed to trying to find the
    // intended non-proxy component for each proxy component, because it is not possible to find the intended
    // non-proxy component in all cases. In particular, if the type of the component is not a USceneComponent,
    // then we can't use the unique parent-child relationship between the proxy and non-proxy component to
    // determine how to re-initialize the proxy component. We could work around this issue with some
    // additional book-keeping data structures, but we prefer the simple solution of doing a full tear-down
    // and re-initialization step.
    //

    if (request_reinitialize_) {
        request_reinitialize_ = false;
        findAndDestroyAllProxyComponents(actors);
        initialize();
    }

    if (IsInitialized()) {
        findAndUnregisterAllProxyComponents(actors);
        findAndRegisterAllProxyComponents(actors);
    }
}

bool ASpProxyComponentManager::ShouldTickIfViewportsOnly() const
{
    return true;
}

void ASpProxyComponentManager::Initialize()
{
    if (IsInitialized()) {
        return;
    }

    initialize();
}

void ASpProxyComponentManager::Terminate()
{
    if (!IsInitialized()) {
        return;
    }

    terminate();
}

void ASpProxyComponentManager::Update()
{
    if (!IsInitialized()) {
        return;
    }

    update();
}

bool ASpProxyComponentManager::IsInitialized() const
{
    return is_initialized_;
}

void ASpProxyComponentManager::initialize()
{
    is_initialized_ = true;
    bIsInitialized = true;
}

void ASpProxyComponentManager::terminate()
{
    is_initialized_ = false;
    bIsInitialized = false;
    unregisterProxyComponents(Std::keys(name_to_proxy_component_desc_map_));
}

void ASpProxyComponentManager::update()
{
    std::vector<AActor*> actors = Unreal::findActors(GetWorld());
    unregisterProxyComponents(Std::keys(name_to_proxy_component_desc_map_));
    findAndUnregisterAllProxyComponents(actors);
    findAndRegisterAllProxyComponents(actors);
}

uint32_t ASpProxyComponentManager::getId(uint32_t initial_guess_id, const std::set<uint32_t>& already_allocated_ids, uint32_t max_id)
{
    SP_ASSERT(initial_guess_id >= 1);
    SP_ASSERT(max_id >= 1);

    // linear search starting from the initial guess up to (and including) the maximum ID
    for (uint32_t i = initial_guess_id; i <= max_id; i++) {
        if (!already_allocated_ids.contains(i)) {
            return i;
        }
    }

    // linear search starting from 1 up to (but excluding) the initial guess
    for (uint32_t i = 1; i < initial_guess_id; i++) {
        if (!already_allocated_ids.contains(i)) {
            return i;
        }
    }

    SP_ASSERT(false); // give up if we couldn't find a free ID
    return 0;
}

std::string ASpProxyComponentManager::getManagerName() const
{
    #if WITH_EDITOR // defined in an auto-generated header
        if (!HasAnyFlags(RF_Transient)) {
            return Unreal::toStdString(GetActorLabel());
        }
    #endif
    return Unreal::toStdString(GetName());
}

std::string ASpProxyComponentManager::getProxyComponentName(uint32_t id) const
{
    // TODO: remove platform-specific logic
    #if BOOST_COMP_MSVC
        return "__SP_PROXY_COMPONENT_" + getManagerName() + "_" + std::format("{:#018x}", id) + "__";
    #elif BOOST_COMP_CLANG
        return "__SP_PROXY_COMPONENT_" + getManagerName() + "_" + (boost::format("0x%016x")%id).str() + "__";
    #else
        #error
    #endif
}

std::string ASpProxyComponentManager::getLongComponentName(const UWorld* world, const UActorComponent* component)
{
    SP_ASSERT(GEngine);
    SP_ASSERT(world);
    SP_ASSERT(component);

    std::string world_type;
    if (world->IsEditorWorld() && !world->IsGameWorld() && !world->IsPreviewWorld() && GEngine->GetWorldContextFromWorld(world)) {
        world_type = "editor";
    } else if (world->IsGameWorld() && !world->IsPreviewWorld() && GEngine->GetWorldContextFromWorld(world)) {
        world_type = "game";
    } else {
        SP_ASSERT(false);
    }

    bool include_actor_name = true;
    bool actor_must_have_stable_name = false;
    std::string actor_and_component_name = Unreal::getStableName(component, include_actor_name, actor_must_have_stable_name);

    return world_type + "://" + actor_and_component_name;
}
