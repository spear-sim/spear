//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpUnrealTypes/SpActorHitManager.h"

#include <map>

#include <Containers/Array.h>
#include <Engine/EngineBaseTypes.h> // ETickingGroup
#include <Engine/HitResult.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>           // uint64
#include <Templates/Casts.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/SpStableNameManager.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealUtils.h"

ASpActorHitManager::ASpActorHitManager()
{
    SP_LOG_CURRENT_FUNCTION();

    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = false;
    PrimaryActorTick.TickGroup = ETickingGroup::TG_PrePhysics;
}

ASpActorHitManager::~ASpActorHitManager()
{
    SP_LOG_CURRENT_FUNCTION();
}

void ASpActorHitManager::Tick(float delta_time)
{
    AActor::Tick(delta_time);
    actor_hit_descs_.clear(); // will be cleared during ETickingGroup::TG_PrePhysics, i.e., before any hit events happen for this frame
}

void ASpActorHitManager::SubscribeToActor(AActor* Actor)
{
    SP_ASSERT(Actor);
    Actor->OnActorHit.AddDynamic(this, &ASpActorHitManager::ActorHitHandler);
}

void ASpActorHitManager::UnsubscribeFromActor(AActor* Actor)
{
    SP_ASSERT(Actor);
    Actor->OnActorHit.RemoveDynamic(this, &ASpActorHitManager::ActorHitHandler);
}

TArray<FActorHitDesc> ASpActorHitManager::GetActorHitDescs(bool bIncludeDebugInfo)
{
    TArray<FActorHitDesc> actor_hit_descs;

    for (auto& desc : actor_hit_descs_) {
        FActorHitDesc actor_hit_desc;

        actor_hit_desc.SelfActor     = reinterpret_cast<uint64>(desc.self_actor_);
        actor_hit_desc.OtherActor    = reinterpret_cast<uint64>(desc.other_actor_);
        actor_hit_desc.NormalImpulse = desc.normal_impulse_;
        actor_hit_desc.HitResult     = desc.hit_result_;

        if (bIncludeDebugInfo) {
            actor_hit_desc.SelfActorPtrString         = Unreal::toFString(Std::toStringFromPtr(desc.self_actor_));
            actor_hit_desc.SelfActorPropertiesString  = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(desc.self_actor_));
            actor_hit_desc.OtherActorPtrString        = Unreal::toFString(Std::toStringFromPtr(desc.other_actor_));
            actor_hit_desc.OtherActorPropertiesString = Unreal::toFString(UnrealUtils::getObjectPropertiesAsString(desc.other_actor_));
        }

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
