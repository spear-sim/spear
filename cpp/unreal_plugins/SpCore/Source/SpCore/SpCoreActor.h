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
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <UObject/NameTypes.h>           // FName
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS

#include "SpCore/CppFuncComponent.h"
#include "SpCore/StableNameComponent.h"

#include "SpCoreActor.generated.h"

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
class SPCORE_API ASpCoreActor : public AActor
{
    GENERATED_BODY()
public: 
    ASpCoreActor();
    ~ASpCoreActor();

    // AActor interface
    void Tick(float delta_time) override;

    // Required for keeping StableNameComponents up-to-date.
    #if WITH_EDITOR // defined in an auto-generated header
        void PostActorCreated() override;
        void PostLoad() override;
        void BeginDestroy() override;
    #endif

private:
    // Required so this actor can be found through our Unreal::findActor interface.
    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "Stable Name Component");
    UStableNameComponent* StableNameComponent = nullptr;

    // Used for debugging.
    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName = "CPP Func Component");
    UCppFuncComponent* CppFuncComponent = nullptr;

    // Calling UGameplayStatics::SetGamePaused() doesn't synchronize with the default play/pause button in the editor
    // So we provide custom buttons and a custom read-only property to update and visualize the engine-level (as
    // opposed to the editor-level) pause state of the game. This functionality is implemented by interacting directly
    // with UGameplayStatics, and is therefore guaranteed to be synchronized with the engine-level state.
    UFUNCTION(CallInEditor, Category="SPEAR")
    void PauseGame();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void UnpauseGame();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void ToggleGamePaused();

    UPROPERTY(VisibleAnywhere, Category = "SPEAR", DisplayName="Is Game Paused");
    bool IsGamePaused = false;

    // Interface for subscribing to, unsubscribing from, and getting actor hit events. Part of this interface must
    // be implemented in terms of a UFUNCTION. We choose to implement the rest of the interface directly in this
    // actor so we can keep the entire interface in one place in the code, near the required UFUNCTION. An alternative
    // would be to implement this interface as a collection of entry points on a service (e.g., GameWorldService).
    UFUNCTION()
    void SubscribeToActorHitEvents(AActor* actor);
    UFUNCTION()
    void UnsubscribeFromActorHitEvents(AActor* actor);
    UFUNCTION()
    TArray<FActorHitEventDesc> GetActorHitEventDescs();
    UFUNCTION()
    void ActorHitHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result);
    TArray<FActorHitEventDesc> actor_hit_event_descs_;

    // Required for keeping StableNameComponents up-to-date.
    #if WITH_EDITOR // defined in an auto-generated header
        void initializeActorLabelHandlers();
        void requestTerminateActorLabelHandlers();

        void actorLabelChangedHandler(AActor* actor);
        void levelActorFolderChangedHandler(const AActor* actor, FName name);

        FDelegateHandle actor_label_changed_handle_;
        FDelegateHandle level_actor_folder_changed_handle_;
    #endif

    // The UPROPERTIES below are required to support the Unreal::findSpecialStructByName(...) method.
    UPROPERTY()
    FRotator _SP_SPECIAL_STRUCT_FRotator_;
    UPROPERTY()
    FVector _SP_SPECIAL_STRUCT_FVector_;
};
