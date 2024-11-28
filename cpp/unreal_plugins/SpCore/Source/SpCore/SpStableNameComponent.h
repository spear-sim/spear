//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <Components/ActorComponent.h>
#include <Containers/UnrealString.h>     // FString
#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <GameFramework/Actor.h>
#include <HAL/Platform.h>                // SPCORE_API
#include <UObject/NameTypes.h>           // FName
#include <UObject/ObjectMacros.h>        // GENERATED_BODY, UCLASS

#include "SpStableNameComponent.generated.h"

//
// If a USpStableNameComponent is attached to an actor, then it can be found using its name and path in the
// Unreal Editor Outliner view, even in standalone shipping builds, using Unreal::findActorByName(...) and
// similar functions.
//

// We need meta=(BlueprintSpawnableComponent) for the component to show up when using the "+Add" button in the editor.
UCLASS(ClassGroup="SPEAR", HideCategories=(Rendering, Tags, Activation, Cooking, Physics, LOD, AssetUserData, Collision), meta=(BlueprintSpawnableComponent))
class SPCORE_API USpStableNameComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    USpStableNameComponent();
    ~USpStableNameComponent();

    #if WITH_EDITOR // defined in an auto-generated header
        // UActorComponent interface
        void OnComponentCreated() override;
        void PostLoad() override;

        // Interface for updating StableName if the component isn't spawned at runtime.
        void requestUpdate();
    #endif

    UPROPERTY(VisibleAnywhere, Category="SPEAR");
    FString StableName;
};

//
// The ASpStableNameManager default object listens to UI events in the editor and keeps USpStableNameComponents
// up-to-date.
//

UCLASS()
class ASpStableNameManager : public AActor
{
    GENERATED_BODY()
public: 
    ASpStableNameManager();
    ~ASpStableNameManager();

#if WITH_EDITOR // defined in an auto-generated header
    private:
        void postEngineInitHandler();
        void enginePreExitHandler();
        void actorLabelChangedHandler(AActor* actor);
        void levelActorFolderChangedHandler(const AActor* actor, FName name);

        FDelegateHandle post_engine_init_handle_;
        FDelegateHandle engine_pre_exit_handle_;
        FDelegateHandle actor_label_changed_handle_;
        FDelegateHandle level_actor_folder_changed_handle_;
#endif
};
