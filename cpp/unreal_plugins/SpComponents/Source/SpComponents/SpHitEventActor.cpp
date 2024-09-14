//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpComponents/SpHitEventActor.h"

#include <Containers/Array.h>
#include <Engine/EngineBaseTypes.h> // ETickingGroup
#include <Engine/EngineTypes.h>     // FHitResult
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>           // uint64

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Unreal.h"

ASpHitEventActor::ASpHitEventActor()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = false;
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;

    // USpStableNameComponent
    SpStableNameComponent = Unreal::createComponentInsideOwnerConstructor<USpStableNameComponent>(this, "sp_stable_name_component");
    SP_ASSERT(SpStableNameComponent);
}

ASpHitEventActor::~ASpHitEventActor()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpHitEventActor::Tick(float delta_time)
{
    AActor::Tick(delta_time);
    actor_hit_event_descs_.Empty();
}

void ASpHitEventActor::SubscribeToActor(AActor* Actor)
{
    Actor->OnActorHit.AddDynamic(this, &ASpHitEventActor::ActorHitHandler);
}

void ASpHitEventActor::UnsubscribeFromActor(AActor* Actor)
{
    Actor->OnActorHit.AddDynamic(this, &ASpHitEventActor::ActorHitHandler);
}

TArray<FActorHitEventDesc> ASpHitEventActor::GetHitEventDescs()
{
    return actor_hit_event_descs_;
}

void ASpHitEventActor::ActorHitHandler(AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& HitResult)
{
    SP_ASSERT(SelfActor);
    SP_ASSERT(OtherActor);

    FActorHitEventDesc actor_hit_event_desc;
    actor_hit_event_desc.SelfActor = reinterpret_cast<uint64>(SelfActor);
    actor_hit_event_desc.OtherActor = reinterpret_cast<uint64>(OtherActor);
    actor_hit_event_desc.NormalImpulse = NormalImpulse;
    actor_hit_event_desc.HitResult = HitResult;

    if (bStoreDebugInfo) {
        actor_hit_event_desc.SelfActorDebugPtr = Unreal::toFString(Std::toStringFromPtr(SelfActor));
        actor_hit_event_desc.SelfActorDebugInfo = Unreal::toFString(Unreal::getObjectPropertiesAsString(SelfActor));
        actor_hit_event_desc.OtherActorDebugPtr = Unreal::toFString(Std::toStringFromPtr(OtherActor));
        actor_hit_event_desc.OtherActorDebugInfo = Unreal::toFString(Unreal::getObjectPropertiesAsString(OtherActor));
    }

    actor_hit_event_descs_.Add(actor_hit_event_desc);
}
