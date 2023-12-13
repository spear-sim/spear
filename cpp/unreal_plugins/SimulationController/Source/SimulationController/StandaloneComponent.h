//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <type_traits> // is_base_of

#include <Components/SceneComponent.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>

#include "CoreUtils/Assert.h"

template <typename TComponent>
class StandaloneComponent
{
public:
    StandaloneComponent() = delete;

    StandaloneComponent(UWorld* world, const std::string& name)
    {
        actor_ = world->SpawnActor<AActor>();
        SP_ASSERT(actor_);

        component_ = createComponent<std::is_base_of<USceneComponent, TComponent>::value>(actor_, actor_, name);
        SP_ASSERT(component_);
    }

    ~StandaloneComponent()
    {
        // Objects created with CreateDefaultSubobject, DuplicateObject, LoadObject, NewObject don't need to be cleaned up explicitly.

        SP_ASSERT(component_);
        component_ = nullptr;

        SP_ASSERT(actor_);
        actor_->Destroy();
        actor_ = nullptr;
    }

    TComponent* component_ = nullptr;

private:
    template <bool is_scene_component>
    TComponent* createComponent(UObject* owner, AActor* parent, const std::string& name)
    {
        SP_ASSERT(false);
        return nullptr;
    }

    template <>
    TComponent* createComponent<true>(UObject* owner, AActor* parent, const std::string& name)
    {
        return Unreal::createSceneComponentOutsideOwnerConstructor<TComponent>(actor_, actor_, name);
    }

    template <>
    TComponent* createComponent<false>(UObject* owner, AActor* parent, const std::string& name)
    {
        return Unreal::createComponentOutsideOwnerConstructor<TComponent>(actor_, name);
    }

    AActor* actor_ = nullptr;
};
