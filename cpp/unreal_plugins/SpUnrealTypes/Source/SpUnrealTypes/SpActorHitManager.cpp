//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpActorHitManager.h"

#include <Containers/Array.h>
#include <Engine/EngineBaseTypes.h> // ETickingGroup
#include <Engine/HitResult.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>           // uint64
#include <Templates/Casts.h>

#include "SpCore/Assert.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

ASpActorHitManager::ASpActorHitManager()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = false;
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

void ASpActorHitManager::Tick(float delta_time)
{
    AActor::Tick(delta_time);
    actor_hit_descs_.clear(); // will be cleared during ETickingGroup::TG_PrePhysics, i.e., before any hit events happen for this frame
}

void ASpActorHitManager::SubscribeToActor(AActor* Actor)
{
    SP_ASSERT(Unreal::isValid(Actor));
    Actor->OnActorHit.AddDynamic(this, &ASpActorHitManager::ActorHitHandler);
}

void ASpActorHitManager::UnsubscribeFromActor(AActor* Actor)
{
    SP_ASSERT(Unreal::isValid(Actor));
    Actor->OnActorHit.RemoveDynamic(this, &ASpActorHitManager::ActorHitHandler);
}

TArray<FSpActorHitDesc> ASpActorHitManager::GetActorHitDescs(bool bIncludeDebugInfo) const
{
    TArray<FSpActorHitDesc> actor_hit_descs;

    for (const auto& desc : actor_hit_descs_) {
        FSpActorHitDesc actor_hit_desc;

        AActor* self_actor = Unreal::toPtr(desc.self_actor_);
        SP_ASSERT(self_actor);
        actor_hit_desc.SelfActor = Unreal::toFString(UnrealUtils::toStringFromPtr(self_actor));
        if (bIncludeDebugInfo) {
            actor_hit_desc.SelfActorPropertiesString = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(self_actor));
        }

        AActor* other_actor = Unreal::toPtr(desc.other_actor_);
        actor_hit_desc.OtherActor = Unreal::toFString(UnrealUtils::toStringFromPtr(other_actor));
        if (bIncludeDebugInfo && other_actor) {
            actor_hit_desc.OtherActorPropertiesString = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(other_actor));
        }

        actor_hit_desc.NormalImpulse = desc.normal_impulse_;
        actor_hit_desc.HitResult = desc.hit_result_;

        actor_hit_descs.Add(actor_hit_desc);
    }

    return actor_hit_descs;
}

void ASpActorHitManager::ActorHitHandler(AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& HitResult)
{
    SP_ASSERT(SelfActor);
    SP_ASSERT(OtherActor);

    ActorHitDesc actor_hit_desc;

    actor_hit_desc.self_actor_     = SelfActor;
    actor_hit_desc.other_actor_    = OtherActor;
    actor_hit_desc.normal_impulse_ = NormalImpulse;
    actor_hit_desc.hit_result_     = HitResult;

    actor_hit_descs_.push_back(actor_hit_desc);
}
