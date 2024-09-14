//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpStableNameComponent.h"

#include <Components/ActorComponent.h>
#include <GameFramework/Actor.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

USpStableNameComponent::USpStableNameComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

USpStableNameComponent::~USpStableNameComponent()
{
    SP_LOG_CURRENT_FUNCTION();
}

#if WITH_EDITOR // defined in an auto-generated header
    void USpStableNameComponent::OnComponentCreated()
    {
        UActorComponent::OnComponentCreated();
        requestUpdate();
    }

    void USpStableNameComponent::PostLoad()
    {
        UActorComponent::PostLoad();
        requestUpdate();
    }

    void USpStableNameComponent::requestUpdate()
    {
        AActor* actor = GetOwner();
        SP_ASSERT(actor);

        // This method will not update the stable name of any actor spawned at runtime. Any such actor
        // needs to update its stable name via Unreal::setStableName(...).
        if (!actor->HasAnyFlags(RF_Transient)) {
            FName folder_path = actor->GetFolderPath();
            if (folder_path.IsNone()) {
                StableName = Unreal::toFString(Unreal::toStdString(actor->GetActorLabel()));
            } else {
                StableName = Unreal::toFString(Unreal::toStdString(folder_path) + "/" + Unreal::toStdString(actor->GetActorLabel()));
            }
        }
    }
#endif
