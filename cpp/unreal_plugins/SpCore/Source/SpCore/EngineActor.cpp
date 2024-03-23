//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/EngineActor.h"

#include <Engine/Engine.h>      // GEngine
#include <GameFramework/Actor.h>
#include <Misc/CoreDelegates.h>
#include <UObject/Class.h>      // UStruct
#include <UObject/NameTypes.h>  // FName
#include <UObject/UnrealType.h> // FProperty, FStructProperty

#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

AEngineActor::AEngineActor()
{
    SP_LOG_CURRENT_FUNCTION();
}

AEngineActor::~AEngineActor()
{
    SP_LOG_CURRENT_FUNCTION();
}

#if WITH_EDITOR
    void AEngineActor::PostActorCreated()
    {
        AActor::PostActorCreated();
        initializeHandlers();
    }

    void AEngineActor::PostLoad()
    {
        AActor::PostLoad();
        initializeHandlers();
    }

    void AEngineActor::BeginDestroy()
    {
        AActor::BeginDestroy();
        terminateHandlers();
    }

    void AEngineActor::initializeHandlers()
    {
        SP_ASSERT(GEngine);
        SP_ASSERT(!actor_label_changed_handle_.IsValid());
        SP_ASSERT(!level_actor_folder_changed_handle_.IsValid());
        actor_label_changed_handle_ = FCoreDelegates::OnActorLabelChanged.AddUObject(this, &AEngineActor::actorLabelChangedHandler);
        level_actor_folder_changed_handle_ = GEngine->OnLevelActorFolderChanged().AddUObject(this, &AEngineActor::levelActorFolderChangedHandler);
    }

    void AEngineActor::terminateHandlers()
    {
        // Need to check IsValid() here because BeginDestroy() is called for default objects, but PostActorCreated() and PostLoad() are not.

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

    void AEngineActor::actorLabelChangedHandler(AActor* actor)
    {
        SP_ASSERT(actor);
        Unreal::requestUpdateStableActorName(actor);
    }

    void AEngineActor::levelActorFolderChangedHandler(const AActor* actor, FName name)
    {
        SP_ASSERT(actor);
        Unreal::requestUpdateStableActorName(actor);
    }
#endif

UStruct* AEngineActor::findStaticStructByName(const std::string& struct_name)
{
    Unreal::PropertyDesc property_desc = Unreal::findPropertyByName(this, struct_name);
    SP_ASSERT(property_desc.property_);
    SP_ASSERT(property_desc.property_->IsA(FStructProperty::StaticClass()));
    FStructProperty* struct_property = static_cast<FStructProperty*>(property_desc.property_);
    return struct_property->Struct;
}
