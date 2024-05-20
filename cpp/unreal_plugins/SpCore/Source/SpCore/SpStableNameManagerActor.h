//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <GameFramework/Actor.h>
#include <UObject/NameTypes.h>           // FName
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS

#include "SpStableNameManagerActor.generated.h"

UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ASpStableNameManagerActor : public AActor
{
    GENERATED_BODY()
public: 
    ASpStableNameManagerActor();
    ~ASpStableNameManagerActor();

    // AActor interface
    #if WITH_EDITOR // defined in an auto-generated header
        void PostActorCreated() override;
        void PostLoad() override;
        void BeginDestroy() override;
    #endif

private:
    #if WITH_EDITOR // defined in an auto-generated header
        void initializeActorLabelHandlers();
        void requestTerminateActorLabelHandlers();

        void actorLabelChangedHandler(AActor* actor);
        void levelActorFolderChangedHandler(const AActor* actor, FName name);

        FDelegateHandle actor_label_changed_handle_;
        FDelegateHandle level_actor_folder_changed_handle_;
    #endif
};
