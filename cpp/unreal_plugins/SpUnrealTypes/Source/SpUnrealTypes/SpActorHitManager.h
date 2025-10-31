//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <vector>

#include <Containers/Array.h>
#include <Containers/UnrealString.h> // FString
#include <Engine/HitResult.h>
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>            // uint64
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h>    // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "SpActorHitManager.generated.h"

class USpStableNameComponent;

USTRUCT()
struct FActorHitDesc
{
    GENERATED_BODY()

    UPROPERTY()
    uint64 SelfActor = 0;
    UPROPERTY()
    uint64 OtherActor = 0;
    UPROPERTY()
    FVector NormalImpulse = FVector::ZeroVector;
    UPROPERTY()
    FHitResult HitResult;

    // Optional debug info
    UPROPERTY()
    FString SelfActorPtrString;
    UPROPERTY()
    FString SelfActorPropertiesString;
    UPROPERTY()
    FString OtherActorPtrString;
    UPROPERTY()
    FString OtherActorPropertiesString;
};

UCLASS()
class ASpActorHitManager : public AActor
{
    GENERATED_BODY()
public: 
    ASpActorHitManager();
    ~ASpActorHitManager() override;

    // AActor interface
    void Tick(float delta_time) override;

    UFUNCTION()
    void SubscribeToActor(AActor* Actor);

    UFUNCTION()
    void UnsubscribeFromActor(AActor* Actor);

    UFUNCTION()
    TArray<FActorHitDesc> GetActorHitDescs(bool bIncludeDebugInfo = false);

private:
    UFUNCTION() // needs to be a UFUNCTION
    void ActorHitHandler(AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& HitResult);

    struct ActorHitDesc
    {
        AActor* self_actor_ = nullptr;
        AActor* other_actor_ = nullptr;
        FVector normal_impulse_ = FVector::ZeroVector;
        FHitResult hit_result_;
    };

    std::vector<ActorHitDesc> actor_hit_descs_;
};
