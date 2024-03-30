//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/EngineActor.h"

#include <Containers/Array.h>
#include <Engine/Engine.h>      // GEngine
#include <Engine/EngineTypes.h> // FHitResult
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>       // uint64
#include <Kismet/GameplayStatics.h>
#include <Math/Vector.h>
#include <Misc/CoreDelegates.h>
#include <UObject/NameTypes.h>  // FName

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/StableNameComponent.h"
#include "SpCore/Unreal.h"

AEngineActor::AEngineActor()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = false;
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;

    stable_name_component_ = Unreal::createComponentInsideOwnerConstructor<UStableNameComponent>(this, "stable_name");
    SP_ASSERT(stable_name_component_);
}

AEngineActor::~AEngineActor()
{
    SP_LOG_CURRENT_FUNCTION();
}

void AEngineActor::Tick(float delta_time)
{
    AActor::Tick(delta_time);

    IsGamePaused = UGameplayStatics::IsGamePaused(GetWorld());

    actor_hit_event_descs_.Empty();
}

void AEngineActor::PauseGame()
{
    UGameplayStatics::SetGamePaused(GetWorld(), true);
}

void AEngineActor::UnpauseGame()
{
    UGameplayStatics::SetGamePaused(GetWorld(), false);
}

void AEngineActor::ToggleGamePaused()
{
    UGameplayStatics::SetGamePaused(GetWorld(), !UGameplayStatics::IsGamePaused(GetWorld()));
}

void AEngineActor::SubscribeToActorHitEvents(AActor* actor)
{
    actor->OnActorHit.AddDynamic(this, &AEngineActor::ActorHitHandler);
}

void AEngineActor::UnsubscribeFromActorHitEvents(AActor* actor)
{
    actor->OnActorHit.AddDynamic(this, &AEngineActor::ActorHitHandler);
}

TArray<FActorHitEventDesc> AEngineActor::GetActorHitEventDescs()
{
    return actor_hit_event_descs_;
}

void AEngineActor::ActorHitHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
{
    SP_ASSERT(self_actor);
    SP_ASSERT(other_actor);

    FActorHitEventDesc actor_hit_event_desc;
    actor_hit_event_desc.SelfActor = reinterpret_cast<uint64>(self_actor);
    actor_hit_event_desc.OtherActor = reinterpret_cast<uint64>(other_actor);
    actor_hit_event_desc.NormalImpulse = normal_impulse;
    actor_hit_event_desc.HitResult = hit_result;

    actor_hit_event_desc.SelfActorDebugPtr = Unreal::toFString(Std::toStringFromPtr(self_actor));
    actor_hit_event_desc.SelfActorDebugInfo = Unreal::toFString(Unreal::getObjectPropertiesAsString(self_actor));
    actor_hit_event_desc.OtherActorDebugPtr = Unreal::toFString(Std::toStringFromPtr(other_actor));
    actor_hit_event_desc.OtherActorDebugInfo = Unreal::toFString(Unreal::getObjectPropertiesAsString(other_actor));

    actor_hit_event_descs_.Add(actor_hit_event_desc);

    // HACK: Strictly speaking, this code doesn't need to be here, but I want to test this function when the array of hit events is non-empty.
    UFunction* ufunction = Unreal::findFunctionByName(this->GetClass(), "GetActorHitEventDescs");
    std::map<std::string, std::string> return_values = Unreal::callFunction(this, ufunction);
    for (auto& [return_value_name, return_value] : return_values) {
        SP_LOG(return_value_name);
        SP_LOG(return_value);
    }
}

#if WITH_EDITOR // defined in an auto-generated header
    void AEngineActor::PostActorCreated()
    { 
        AActor::PostActorCreated();
        initializeActorLabelHandlers();
    }

    void AEngineActor::PostLoad()
    {
        AActor::PostLoad();
        initializeActorLabelHandlers();
    }

    void AEngineActor::BeginDestroy()
    {
        AActor::BeginDestroy();
        requestTerminateActorLabelHandlers();
    }

    void AEngineActor::initializeActorLabelHandlers()
    {
        SP_ASSERT(GEngine);
        SP_ASSERT(!actor_label_changed_handle_.IsValid());
        SP_ASSERT(!level_actor_folder_changed_handle_.IsValid());
        actor_label_changed_handle_ = FCoreDelegates::OnActorLabelChanged.AddUObject(this, &AEngineActor::actorLabelChangedHandler);
        level_actor_folder_changed_handle_ = GEngine->OnLevelActorFolderChanged().AddUObject(this, &AEngineActor::levelActorFolderChangedHandler);
    }

    void AEngineActor::requestTerminateActorLabelHandlers()
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
