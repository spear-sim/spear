//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>

#include <Containers/Array.h>
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/EngineTypes.h>          // FHitResult
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>                // uint64
#include <Math/Vector.h>
#include <UObject/NameTypes.h>           // FName
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS

#include "EngineActor.generated.h"

class UStableNameComponent;

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
class SPCORE_API AEngineActor : public AActor
{
    GENERATED_BODY()
public: 
    AEngineActor();
    ~AEngineActor();

    // AActor interface
    void Tick(float delta_time) override;

    // Required for keeping StableNameComponents up-to-date.
    #if WITH_EDITOR // defined in an auto-generated header
        void PostActorCreated() override;
        void PostLoad() override;
        void BeginDestroy() override;
    #endif

private:
    // Calling UGameplayStatics::SetGamePaused() doesn't synchronize with the default play/pause button in the
    // editor. So we provide custom buttons and a custom read-only property to visualize and update the pause
    // state of the game, as defined by UGameplayStatics.
    UFUNCTION(CallInEditor, Category="SPEAR")
    void PauseGame();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void UnpauseGame();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void ToggleGamePaused();

    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName="Is Game Paused");
    bool IsGamePaused = false;

    // Interface for subscribing to, unsubscribing from, and getting actor hit events.
    UFUNCTION()
    void SubscribeToActorHitEvents(AActor* actor);

    UFUNCTION()
    void UnsubscribeFromActorHitEvents(AActor* actor);

    UFUNCTION()
    TArray<FActorHitEventDesc> GetActorHitEventDescs();

    UFUNCTION()
    void ActorHitHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result);

    TArray<FActorHitEventDesc> actor_hit_event_descs_;

    // Used for obtaining a UStruct* in cases where a class or struct doesn't define a StaticStruct() function.
    UPROPERTY()
    FVector FVector_;

    // Required for keeping StableNameComponents up-to-date.
    #if WITH_EDITOR // defined in an auto-generated header
        void initializeActorLabelHandlers();
        void requestTerminateActorLabelHandlers();

        void actorLabelChangedHandler(AActor* actor);
        void levelActorFolderChangedHandler(const AActor* actor, FName name);

        FDelegateHandle actor_label_changed_handle_;
        FDelegateHandle level_actor_folder_changed_handle_;
    #endif

    UStableNameComponent* stable_name_component_ = nullptr;
};
