//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/SpFuncService.h"

#include <Components/SceneComponent.h>
#include <Engine/Engine.h>  // GEngine
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <UObject/Object.h> // UObject

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

#include "SpComponents/SpFuncComponent.h"

void SpFuncService::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
        SP_ASSERT(!world_);
        world_ = world;
    }
}

void SpFuncService::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    if (world == world_) {
        world_ = nullptr;
    }
}

USpFuncComponent* SpFuncService::getSpFuncComponent(const UObject* uobject)
{
    USpFuncComponent* sp_func_component = nullptr;
    if (uobject->IsA(AActor::StaticClass())) {
        AActor* actor = const_cast<AActor*>(static_cast<const AActor*>(uobject));
        bool include_all_descendants = false;
        sp_func_component = Unreal::getChildComponentByType<AActor, USpFuncComponent>(actor, include_all_descendants);
    } else if (uobject->IsA(USceneComponent::StaticClass())) {
        USceneComponent* component = const_cast<USceneComponent*>(static_cast<const USceneComponent*>(uobject));
        bool include_all_descendants = false;
        sp_func_component = Unreal::getChildComponentByType<USceneComponent, USpFuncComponent>(component, include_all_descendants);
    } else {
        SP_ASSERT(false);
    }
    SP_ASSERT(sp_func_component);
    return sp_func_component;
}
