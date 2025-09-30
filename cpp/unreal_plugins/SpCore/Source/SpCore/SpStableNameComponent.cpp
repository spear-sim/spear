//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/SpStableNameComponent.h"

#include <Components/ActorComponent.h>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <GameFramework/Actor.h>
#include <Misc/CoreDelegates.h>
#include <UObject/NameTypes.h>           // FName

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

//
// USpStableNameComponent
//

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
        if (!actor) {
            return;
        }

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

//
// USpStableNameManager
//

USpStableNameManager::USpStableNameManager()
{
    SP_LOG_CURRENT_FUNCTION();

    #if WITH_EDITOR // defined in an auto-generated header
        post_engine_init_handle_ = FCoreDelegates::OnPostEngineInit.AddUObject(this, &USpStableNameManager::postEngineInitHandler);
        engine_pre_exit_handle_ = FCoreDelegates::OnEnginePreExit.AddUObject(this, &USpStableNameManager::enginePreExitHandler);
    #endif
}

USpStableNameManager::~USpStableNameManager()
{
    SP_LOG_CURRENT_FUNCTION();

    #if WITH_EDITOR // defined in an auto-generated header
        FCoreDelegates::OnEnginePreExit.Remove(engine_pre_exit_handle_);
        FCoreDelegates::OnPostEngineInit.Remove(post_engine_init_handle_);

        engine_pre_exit_handle_.Reset();
        post_engine_init_handle_.Reset();
    #endif
}

#if WITH_EDITOR // defined in an auto-generated header
    void USpStableNameManager::postEngineInitHandler()
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(GEngine);
        
        actor_label_changed_handle_ = FCoreDelegates::OnActorLabelChanged.AddUObject(this, &USpStableNameManager::actorLabelChangedHandler);
        level_actor_folder_changed_handle_ = GEngine->OnLevelActorFolderChanged().AddUObject(this, &USpStableNameManager::levelActorFolderChangedHandler);    
    }

    void USpStableNameManager::enginePreExitHandler()
    {
        SP_LOG_CURRENT_FUNCTION();
        SP_ASSERT(GEngine);

        GEngine->OnLevelActorFolderChanged().Remove(level_actor_folder_changed_handle_);
        FCoreDelegates::OnActorLabelChanged.Remove(actor_label_changed_handle_);

        level_actor_folder_changed_handle_.Reset();
        actor_label_changed_handle_.Reset();
    }

    void USpStableNameManager::actorLabelChangedHandler(AActor* actor)
    {
        SP_ASSERT(actor);
        Unreal::requestUpdateStableName(actor);
    }

    void USpStableNameManager::levelActorFolderChangedHandler(const AActor* actor, FName name)
    {
        SP_ASSERT(actor);
        Unreal::requestUpdateStableName(actor);
    }
#endif
