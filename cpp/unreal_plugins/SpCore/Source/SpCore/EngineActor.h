//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <string>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <GameFramework/Actor.h>
#include <UObject/NameTypes.h>           // FName
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS

#include "EngineActor.generated.h"

class UStruct;

UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class SPCORE_API AEngineActor : public AActor
{
    GENERATED_BODY()
public: 
    AEngineActor();
    ~AEngineActor();

    // Required for keeping StableNameComponents up-to-date.
    #if WITH_EDITOR // defined in an auto-generated header
        // AActor interface
        void PostActorCreated() override;
        void PostLoad() override;
        void BeginDestroy() override;
    #endif

private:
    // Required for keeping StableNameComponents up-to-date.
    #if WITH_EDITOR // defined in an auto-generated header
        void initializeHandlers();
        void terminateHandlers();

        void actorLabelChangedHandler(AActor* actor);
        void levelActorFolderChangedHandler(const AActor* actor, FName name);

        FDelegateHandle actor_label_changed_handle_;
        FDelegateHandle level_actor_folder_changed_handle_;
    #endif

    // Used for obtaining a UStruct* in cases where a class or struct
    // doesn't define a StaticStruct() function, e.g., FVector.
    UPROPERTY()
    FVector FVector;
};
