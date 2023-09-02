//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/ActorComponent.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>

#include "CoreUtils/Assert.h"

template <typename TComponent>
class Component
{
public:
    Component() = delete;

    Component(UWorld* world)
    {
        actor_ = world->SpawnActor<AActor>();
        SP_ASSERT(actor_);

        component_ = NewObject<TComponent>(actor_);
        SP_ASSERT(component_);
        component_->RegisterComponent();
    }

    ~Component()
    {
        // Objects created with LoadObject and NewObject don't need to be cleaned up explicitly.

        SP_ASSERT(component_);
        component_ = nullptr;

        SP_ASSERT(actor_);
        actor_->Destroy();
        actor_ = nullptr;
    }

    TComponent* component_ = nullptr;

private:
    AActor* actor_ = nullptr;
};
