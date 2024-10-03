//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>

#include <Containers/Array.h>
#include <Containers/UnrealString.h> // FString
#include <Engine/EngineTypes.h>      // FHitResult
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // uint64
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpCore/SpStableNameComponent.h"

#include "SpHitEventManager.generated.h"

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

    // Optional debug info
    UPROPERTY()
    FString SelfActorPtr;
    UPROPERTY()
    FString SelfActorPropertiesString;
    UPROPERTY()
    FString OtherActorPtr;
    UPROPERTY()
    FString OtherActorPropertiesString;
};

UCLASS()
class ASpHitEventManager : public AActor
{
    GENERATED_BODY()
public: 
    ASpHitEventManager();
    ~ASpHitEventManager();

    // AActor interface
    void Tick(float delta_time) override;

    // Interface for subscribing to, unsubscribing from, and getting actor hit events. Part of this interface
    // (ActorHitHandler) must be implemented as a UFUNCTION. We choose to implement the rest of the interface
    // directly in this actor so we can keep the entire interface in one place in the code, near the required
    // UFUNCTION.

    UFUNCTION()
    static void SubscribeToActor(AActor* Actor, bool bRecordDebugInfo);

    UFUNCTION()
    static void UnsubscribeFromActor(AActor* Actor);

    UFUNCTION()
    static TArray<FActorHitEventDesc> GetHitEventDescs();

private:
    UFUNCTION() // needs to be a UFUNCTION
    void ActorHitHandler(AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& HitResult);
};
