//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Containers/UnrealString.h>     // FString
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <GameFramework/Actor.h>
#include <UObject/NameTypes.h>           // FName
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS, UFUNCTION, UPROPERTY

#include "DebugWidget.generated.h"

UCLASS(Config=Spear, HideCategories=(Rendering, Replication, Collision, HLOD, Physics, Networking, Input, Actor, Cooking))
class ADebugWidget : public AActor
{
    GENERATED_BODY()
public: 
    ADebugWidget();
    ~ADebugWidget();

    // TODO: We override these functions to subscribe/unsubscribe to/from events that would affect an actor's
    // stable name, e.g., renaming it or moving it in the World Outliner. Going forward, we should do this in a
    // more central location. In principle, SimulationController would be a better place to do this, but most
    // functionality in SimulationController doesn't execute in the editor, whereas the events we're subscribing
    // to only broadcast in the editor.
    #if WITH_EDITOR // defined in an auto-generated header
        void PostLoad() override;
        void BeginDestroy() override;
    #endif

    UFUNCTION(CallInEditor, Category="SPEAR")
    void LoadConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SaveConfig();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void PrintDebugString();

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="Debug string")
    FString DebugString;

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SpawnVehiclePawn();

    UFUNCTION(CallInEditor, Category="SPEAR")
    void SpawnUrdfRobotPawn();

    UPROPERTY(EditAnywhere, Config, Category="SPEAR", DisplayName="URDF file")
    FString UrdfFile;

private:
    #if WITH_EDITOR // defined in an auto-generated header
        void actorLabelChangedHandler(AActor* actor);
        void levelActorFolderChangedHandler(const AActor* actor, FName name);

        FDelegateHandle actor_label_changed_handle_;
        FDelegateHandle level_actor_folder_changed_handle_;
    #endif
};
