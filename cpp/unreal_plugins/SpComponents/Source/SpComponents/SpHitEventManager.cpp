//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpHitEventManager.h"

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

static std::map<AActor*, bool> s_record_debug_info_map_;
static TArray<FActorHitEventDesc> s_actor_hit_event_descs_;

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
    s_actor_hit_event_descs_.Empty();
}

void ASpHitEventManager::SubscribeToActor(AActor* Actor, bool bRecordDebugInfo)
{
    SP_ASSERT(Actor);

    if (Std::containsKey(s_record_debug_info_map_, Actor)) {
        UnsubscribeFromActor(Actor);
    }

    Actor->OnActorHit.AddDynamic(Cast<ASpHitEventManager>(ASpHitEventManager::StaticClass()->GetDefaultObject()), &ASpHitEventManager::ActorHitHandler); // no RTTI available
    Std::insert(s_record_debug_info_map_, Actor, bRecordDebugInfo);
}

void ASpHitEventManager::UnsubscribeFromActor(AActor* Actor)
{
    SP_ASSERT(Actor);

    Actor->OnActorHit.RemoveDynamic(Cast<ASpHitEventManager>(ASpHitEventManager::StaticClass()->GetDefaultObject()), &ASpHitEventManager::ActorHitHandler); // no RTTI available
    Std::remove(s_record_debug_info_map_, Actor);
}

TArray<FActorHitEventDesc> ASpHitEventManager::GetHitEventDescs()
{
    return s_actor_hit_event_descs_;
}

void ASpHitEventManager::ActorHitHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
{
    SP_ASSERT(self_actor);
    SP_ASSERT(other_actor);
    SP_ASSERT(Std::containsKey(s_record_debug_info_map_, self_actor));

    FActorHitEventDesc actor_hit_event_desc;
    actor_hit_event_desc.SelfActor = reinterpret_cast<uint64>(self_actor);
    actor_hit_event_desc.OtherActor = reinterpret_cast<uint64>(other_actor);
    actor_hit_event_desc.NormalImpulse = normal_impulse;
    actor_hit_event_desc.HitResult = hit_result;

    if (s_record_debug_info_map_.at(self_actor)) {
        actor_hit_event_desc.SelfActorPtr = Unreal::toFString(Std::toStringFromPtr(self_actor));
        actor_hit_event_desc.SelfActorPropertiesString = Unreal::toFString(Unreal::getObjectPropertiesAsString(self_actor));
        actor_hit_event_desc.OtherActorPtr = Unreal::toFString(Std::toStringFromPtr(other_actor));
        actor_hit_event_desc.OtherActorPropertiesString = Unreal::toFString(Unreal::getObjectPropertiesAsString(other_actor));
    }

    s_actor_hit_event_descs_.Add(actor_hit_event_desc);
}
