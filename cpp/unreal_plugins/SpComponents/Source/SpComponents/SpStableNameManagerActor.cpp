//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpStableNameManagerActor.h"

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/Engine.h>               // GEngine
#include <GameFramework/Actor.h>
#include <Misc/CoreDelegates.h>
#include <UObject/NameTypes.h>           // FName

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

ASpStableNameManagerActor::ASpStableNameManagerActor()
{
    SP_LOG_CURRENT_FUNCTION();
}

ASpStableNameManagerActor::~ASpStableNameManagerActor()
{
    SP_LOG_CURRENT_FUNCTION();
}

#if WITH_EDITOR // defined in an auto-generated header
    void ASpStableNameManagerActor::PostActorCreated()
    { 
        AActor::PostActorCreated();
        initializeActorLabelHandlers();
    }

    void ASpStableNameManagerActor::PostLoad()
    {
        AActor::PostLoad();
        initializeActorLabelHandlers();
    }

    void ASpStableNameManagerActor::BeginDestroy()
    {
        AActor::BeginDestroy();
        requestTerminateActorLabelHandlers();
    }

    void ASpStableNameManagerActor::initializeActorLabelHandlers()
    {
        SP_ASSERT(GEngine);
        SP_ASSERT(!actor_label_changed_handle_.IsValid());
        SP_ASSERT(!level_actor_folder_changed_handle_.IsValid());
        actor_label_changed_handle_ = FCoreDelegates::OnActorLabelChanged.AddUObject(this, &ASpStableNameManagerActor::actorLabelChangedHandler);
        level_actor_folder_changed_handle_ = GEngine->OnLevelActorFolderChanged().AddUObject(this, &ASpStableNameManagerActor::levelActorFolderChangedHandler);
    }

    void ASpStableNameManagerActor::requestTerminateActorLabelHandlers()
    {
        // Need to check IsValid() here because BeginDestroy() is called for default objects, but
        // PostActorCreated() and PostLoad() are not.

        if (level_actor_folder_changed_handle_.IsValid()) {
            SP_ASSERT(GEngine);
            GEngine->OnLevelActorFolderChanged().Remove(level_actor_folder_changed_handle_);
            level_actor_folder_changed_handle_.Reset();
        }

        if (actor_label_changed_handle_.IsValid()) {
            FCoreDelegates::OnActorLabelChanged.Remove(actor_label_changed_handle_);
            actor_label_changed_handle_.Reset();
        }
    }

    void ASpStableNameManagerActor::actorLabelChangedHandler(AActor* actor)
    {
        SP_ASSERT(actor);
        Unreal::requestUpdateStableName(actor);
    }

    void ASpStableNameManagerActor::levelActorFolderChangedHandler(const AActor* actor, FName name)
    {
        SP_ASSERT(actor);
        Unreal::requestUpdateStableName(actor);
    }
#endif
