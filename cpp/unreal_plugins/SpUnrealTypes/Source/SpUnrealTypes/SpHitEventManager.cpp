//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpHitEventManager.h"

#include <map>

#include <Containers/Array.h>
#include <Engine/EngineBaseTypes.h> // ETickingGroup
#include <Engine/HitResult.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>           // uint64
#include <Templates/Casts.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SpStableNameComponent.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

ASpHitEventManager::ASpHitEventManager()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = false;
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

ASpHitEventManager::~ASpHitEventManager()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpHitEventManager::Tick(float delta_time)
{
    AActor::Tick(delta_time);
    actor_hit_event_descs_.Empty();
}

void ASpHitEventManager::SubscribeToActor(AActor* Actor, bool bRecordDebugInfo)
{
    SP_ASSERT(Actor);

    if (Std::containsKey(record_debug_info_for_actors_, Actor)) {
        UnsubscribeFromActor(Actor);
    }

    Actor->OnActorHit.AddDynamic(this, &ASpHitEventManager::ActorHitHandler);
    Std::insert(record_debug_info_for_actors_, Actor, bRecordDebugInfo);
}

void ASpHitEventManager::UnsubscribeFromActor(AActor* Actor)
{
    SP_ASSERT(Actor);

    Actor->OnActorHit.RemoveDynamic(this, &ASpHitEventManager::ActorHitHandler);
    Std::remove(record_debug_info_for_actors_, Actor);
}

TArray<FActorHitEventDesc> ASpHitEventManager::GetHitEventDescs()
{
    return actor_hit_event_descs_;
}

void ASpHitEventManager::ActorHitHandler(AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& HitResult)
{
    SP_ASSERT(SelfActor);
    SP_ASSERT(OtherActor);
    SP_ASSERT(Std::containsKey(record_debug_info_for_actors_, SelfActor));

    FActorHitEventDesc actor_hit_event_desc;
    actor_hit_event_desc.SelfActor = reinterpret_cast<uint64>(SelfActor);
    actor_hit_event_desc.OtherActor = reinterpret_cast<uint64>(OtherActor);
    actor_hit_event_desc.NormalImpulse = NormalImpulse;
    actor_hit_event_desc.HitResult = HitResult;

    if (record_debug_info_for_actors_.at(SelfActor)) {
        actor_hit_event_desc.SelfActorPtr = Unreal::toFString(Std::toStringFromPtr(SelfActor));
        actor_hit_event_desc.SelfActorPropertiesString = Unreal::toFString(Unreal::getObjectPropertiesAsString(SelfActor));
        actor_hit_event_desc.OtherActorPtr = Unreal::toFString(Std::toStringFromPtr(OtherActor));
        actor_hit_event_desc.OtherActorPropertiesString = Unreal::toFString(Unreal::getObjectPropertiesAsString(OtherActor));
    }

    actor_hit_event_descs_.Add(actor_hit_event_desc);
}
