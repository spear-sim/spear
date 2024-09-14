//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/Array.h>
#include <Engine/EngineTypes.h>   // FHitResult
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>         // uint64
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/SpStableNameComponent.h"

#include "SpHitEventActor.generated.h"

class USpStableNameComponent;

USTRUCT()
struct FActorHitEventDesc
{
    GENERATED_BODY()

    UPROPERTY()
    uint64 SelfActor;
    UPROPERTY()
    uint64 OtherActor;
    UPROPERTY()
    FVector NormalImpulse;
    UPROPERTY()
    FHitResult HitResult;

    // Debug info
    UPROPERTY()
    FString SelfActorDebugPtr;
    UPROPERTY()
    FString SelfActorDebugInfo;
    UPROPERTY()
    FString OtherActorDebugPtr;
    UPROPERTY()
    FString OtherActorDebugInfo;
};

UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ASpHitEventActor : public AActor
{
    GENERATED_BODY()
public: 
    ASpHitEventActor();
    ~ASpHitEventActor();

    // AActor interface
    void Tick(float delta_time) override;

    // Interface for subscribing to, unsubscribing from, and getting actor hit events. Part of this interface
    // (ActorHitHandler) must be implemented as a UFUNCTION. We choose to implement the rest of the interface
    // directly in this actor so we can keep the entire interface in one place in the code, near the required
    // UFUNCTION.

    UFUNCTION()
    void SubscribeToActor(AActor* Actor);

    UFUNCTION()
    void UnsubscribeFromActor(AActor* Actor);

    UFUNCTION()
    TArray<FActorHitEventDesc> GetHitEventDescs();

    UPROPERTY(VisibleAnywhere, Category="SPEAR", DisplayName="SP Stable Name Component")
    USpStableNameComponent* SpStableNameComponent = nullptr;

private:
    UFUNCTION()
    void ActorHitHandler(AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& HitResult);

    UPROPERTY(EditAnywhere, Category="SPEAR", DisplayName="Store debug info")
    bool bStoreDebugInfo = true;

    TArray<FActorHitEventDesc> actor_hit_event_descs_;
};
