//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include "SpCore/StableNameComponent.h"

#include <GameFramework/Actor.h>

#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

UStableNameComponent::UStableNameComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

UStableNameComponent::~UStableNameComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

#if WITH_EDITOR // defined in an auto-generated header
    void UStableNameComponent::OnComponentCreated()
    {
        USceneComponent::OnComponentCreated();
        update();
    }

    void UStableNameComponent::PostLoad()
    {
        USceneComponent::PostLoad();
        update();
    }

    void UStableNameComponent::update()
    {
        AActor* actor = GetOwner();
        SP_ASSERT(actor);
        FName folder_path = actor->GetFolderPath();
        if (folder_path.IsNone()) {
            StableName = Unreal::toFString(Unreal::toStdString(actor->GetActorLabel()));
            }
        else {
            StableName = Unreal::toFString(Unreal::toStdString(folder_path) + "/" + Unreal::toStdString(actor->GetActorLabel()));
        }
    }
#endif
