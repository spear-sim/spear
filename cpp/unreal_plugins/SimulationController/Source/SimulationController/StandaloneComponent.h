//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <concepts> // std::derived_from

#include <Components/ActorComponent.h>
#include <Components/SceneComponent.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>

#include "SpCore/Assert.h"
#include "SpCore/Unreal.h"

template <CActorComponent TActorComponent>
class StandaloneComponent
{
public:
    StandaloneComponent() = delete;

    StandaloneComponent(UWorld* world, const std::string& name)
    {
        actor_ = world->SpawnActor<AActor>();
        SP_ASSERT(actor_);

        component_ = Unreal::createComponentOutsideOwnerConstructor<TActorComponent>(actor_, name);
        SP_ASSERT(component_);
    }

    ~StandaloneComponent()
    {
        SP_ASSERT(component_);
        component_ = nullptr;

        SP_ASSERT(actor_);
        actor_->Destroy();
        actor_ = nullptr;
    }

    TActorComponent* component_ = nullptr;
    AActor* actor_ = nullptr;
};
