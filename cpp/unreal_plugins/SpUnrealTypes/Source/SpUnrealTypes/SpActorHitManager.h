//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <vector>

#include <Containers/Array.h>
#include <Containers/UnrealString.h>        // FString
#include <Engine/HitResult.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>                   // uint64
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h>           // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY
#include <UObject/WeakObjectPtrTemplates.h> // TWeakObjectPtr

#include "SpActorHitManager.generated.h"

class USpStableNameComponent;

USTRUCT() // not intended to be Blueprint-accessible
struct FSpActorHitDesc
{
    GENERATED_BODY()

    UPROPERTY()
    FString SelfActor = "0x0";
    UPROPERTY()
    FString OtherActor = "0x0";
    UPROPERTY()
    FVector NormalImpulse = FVector::ZeroVector;
    UPROPERTY()
    FHitResult HitResult;

    // Optional debug info
    UPROPERTY()
    FString SelfActorPropertiesString;
    UPROPERTY()
    FString OtherActorPropertiesString;
};

UCLASS(ClassGroup="SPEAR", HideCategories=(Actor, Collision, Cooking, DataLayers, HLOD, Input, LevelInstance, Navigation, Networking, Physics, Rendering, Replication, WorldPartition))
class ASpActorHitManager : public AActor
{
    GENERATED_BODY()
public:
    ASpActorHitManager();
    ~ASpActorHitManager() = default;

    // AActor interface
    void Tick(float delta_time) override;

    UFUNCTION()
    void SubscribeToActor(AActor* Actor); // not intended to be BlueprintCallable

    UFUNCTION()
    void UnsubscribeFromActor(AActor* Actor); // not intended to be BlueprintCallable

    UFUNCTION()
    TArray<FSpActorHitDesc> GetActorHitDescs(bool bIncludeDebugInfo = false) const; // TArray<FSpActorHitDesc> not supported for BlueprintCallable

private:
    UFUNCTION()
    void ActorHitHandler(AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& HitResult); // needs to be a UFUNCTION

    struct ActorHitDesc
    {
        TWeakObjectPtr<AActor> self_actor_;
        TWeakObjectPtr<AActor> other_actor_;
        FVector normal_impulse_ = FVector::ZeroVector;
        FHitResult hit_result_;
    };

    std::vector<ActorHitDesc> actor_hit_descs_;
};
